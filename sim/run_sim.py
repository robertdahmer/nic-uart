#!/usr/bin/env python3
"""
Simula o dongle passivo do NIC-UART no ngspice e verifica se um receptor RS232
consegue ler os caracteres.

Fluxo:
  1. ./nicuart encode  -> rajadas (início/duração) exatamente como o programa
     real enviaria;
  2. monta o envelope + pulsos NLP/TP_IDL e preenche o netlist do dongle;
  3. roda o ngspice;
  4. passa v(out) por um receptor modelado (limiares do MAX3232) e por um
     UART em software com amostragem 16x;
  5. compara bytes, mede margens de tensão e atrasos e gera PNG + resumo.

Uso: python3 sim/run_sim.py --all
     python3 sim/run_sim.py --variant doubler --baud 9600 --text 'show ver\\r'
"""
import argparse
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(HERE, "out")
NICUART = os.path.join(ROOT, "nicuart")

# Limiares de entrada do MAX3232 (datasheet): típicos e pior caso
TH_RISE_TYP, TH_FALL_TYP = 1.5, 1.2
TH_SPACE_WORST = 2.4   # acima disso garante "space" em qualquer unidade
TH_MARK_WORST = 0.6    # abaixo disso garante "mark" em qualquer unidade

NLP_PERIOD = 16e-3     # 10BASE-T: um normal link pulse a cada 16 ms (+-8)
NLP_WIDTH = 100e-9
TPIDL_WIDTH = 300e-9   # pulso positivo de fim de quadro (TP_IDL)
EDGE = 5e-9


def encode(baud, text, trim_ns):
    r = subprocess.run([NICUART, "encode", "-b", str(baud), "--trim", str(trim_ns), text],
                       capture_output=True, text=True, check=True)
    lines = [l for l in r.stdout.splitlines() if l and not l.startswith("#")]
    meta = r.stdout.splitlines()[0]
    end_ns = int(meta.split("end_ns=")[1].split()[0])
    nbytes = int(meta.split("bytes=")[1].split()[0])
    bursts = []
    for l in lines[1:]:
        s, f, w = (int(x) for x in l.split(","))
        bursts.append((s * 1e-9, w * 1e-9, f))
    return bursts, end_ns * 1e-9, nbytes


def pwl(points):
    out, last_t = [], -1.0
    for t, v in points:
        if t <= last_t:
            t = last_t + 1e-12
        out.append(f"{t:.12e} {v:g}")
        last_t = t
    # continuação de linha do SPICE para não estourar o tamanho da linha
    chunks = [" ".join(out[i:i + 8]) for i in range(0, len(out), 8)]
    return "\n+ ".join(chunks)


def pulse_pts(starts, width, pts):
    for t in starts:
        pts += [(t, 0), (t + EDGE, 1), (t + width, 1), (t + width + EDGE, 0)]


def build(variant, baud, text, vline, trim_ns, t_lead, nlp_mode, params=None, tag=None):
    bursts, end, nbytes = encode(baud, text, trim_ns)
    bursts = [(s + t_lead, w, f) for s, w, f in bursts]
    t_end = end + t_lead
    tstop = t_end + 0.3e-3

    env = [(0, 0)]
    pulse_pts([s for s, _, _ in bursts], 0, [])  # (formato)
    for s, w, _ in bursts:
        env += [(s, 0), (s + EDGE, 1), (s + w, 1), (s + w + EDGE, 0)]
    env.append((tstop, 0))

    nlp_t = []
    # TP_IDL depois de cada quadro
    tpidl = [s + w for s, w, _ in bursts]
    if nlp_mode == "worst":
        # pior caso: um NLP no idle antes do texto (falso start bit?) e outro
        # bem no meio do primeiro bit "mark" de dados (bit errado?)
        nlp_t.append(t_lead / 2)
        bit = 1.0 / baud
        for i in range(len(bursts) - 1):
            gap_start = bursts[i][0] + bursts[i][1]
            gap_end = bursts[i + 1][0]
            if gap_end - gap_start > 0.9 * bit:
                nlp_t.append(gap_start + bit / 2 - NLP_WIDTH / 2)
                break
    else:
        t = t_lead / 2
        while t < tstop:
            nlp_t.append(t)
            t += NLP_PERIOD
    nlp = [(0, 0)]
    pulse_pts(sorted(nlp_t), NLP_WIDTH, nlp)
    pts2 = []
    pulse_pts(tpidl, TPIDL_WIDTH, pts2)
    nlp = sorted(nlp + pts2)
    nlp.append((tstop, 0))

    tag = tag or f"{variant}_{baud}_{vline:.1f}V" + (f"_trim{trim_ns}" if trim_ns else "")
    out_dat = os.path.join(OUT, tag + ".dat")
    tmpl = open(os.path.join(HERE, f"dongle_{variant}.cir")).read()
    net = (tmpl.replace("{VOC}", f"{2 * vline:g}")
               .replace("{ENV_PWL}", pwl(env))
               .replace("{NLP_PWL}", pwl(nlp))
               .replace("{TSTOP}", f"{tstop:.6e}")
               .replace("{OUT}", out_dat)
               .replace("{PARAMS}", "".join(f".param {k}={v}\n" for k, v in (params or {}).items())))
    cir = os.path.join(OUT, tag + ".cir")
    with open(cir, "w") as f:
        f.write(net)
    return dict(tag=tag, cir=cir, dat=out_dat, bursts=bursts, t_lead=t_lead, tstop=tstop,
                t_end=t_end, nbytes=nbytes, nlp=nlp_t)


def run_spice(cir):
    r = subprocess.run(["ngspice", "-b", cir], capture_output=True, text=True)
    if r.returncode != 0:
        sys.stderr.write(r.stdout[-3000:] + r.stderr[-3000:])
        raise SystemExit("ngspice falhou")


def schmitt(v, rise, fall):
    out = np.zeros(len(v), dtype=bool)
    state = False
    for i, x in enumerate(v):
        if not state and x > rise:
            state = True
        elif state and x < fall:
            state = False
        out[i] = state
    return out  # True = space


def uart_decode(t, space, baud, t_from):
    """UART 8N1 com amostragem 16x e votação 7/8/9, como um 16550."""
    bit = 1.0 / baud
    def at(tt):
        return space[min(np.searchsorted(t, tt), len(t) - 1)]
    def vote(t0):
        return sum(at(t0 + k * bit / 16) for k in (-1, 0, 1)) >= 2
    res, errors = [], []
    i = np.searchsorted(t, t_from)
    n = len(t)
    while i < n - 1:
        # procura a borda mark -> space
        while i < n - 1 and not (space[i] and not space[i - 1]):
            i += 1
        if i >= n - 1:
            break
        t0 = t[i]
        if not vote(t0 + bit / 2):
            errors.append(("falso start", t0))
            i += 1
            continue
        if t0 + 9.5 * bit > t[-1]:
            break
        v = 0
        for b in range(8):
            if not vote(t0 + (1.5 + b) * bit):
                v |= 1 << b
        if vote(t0 + 9.5 * bit):
            errors.append(("framing", t0))
        res.append((v, t0))
        i = np.searchsorted(t, t0 + 9.5 * bit)
    return res, errors


def analyze(info, baud, text_bytes, plot=True):
    d = np.loadtxt(info["dat"])
    t, vout, venv = d[:, 0], d[:, 1], d[:, 3]
    bit = 1.0 / baud
    # o ngspice às vezes aborta (timestep too small) e ainda sai com código 0
    truncated = t[-1] < 0.98 * info["tstop"]

    sp_typ = schmitt(vout, TH_RISE_TYP, TH_FALL_TYP)
    decoded, errs = uart_decode(t, sp_typ, baud, 0)
    got = bytes(v for v, _ in decoded)

    # margens: tensão no meio de cada bit ideal
    space_mid, mark_mid = [], []
    total_bits = info["nbytes"] * 10
    # pior valor numa janela de +-1/4 de bit em torno do meio (onde um UART
    # 16x amostra), não só no ponto central
    for k in range(total_bits):
        tm = info["t_lead"] + (k + 0.5) * bit
        lo = np.searchsorted(t, tm - bit / 4)
        hi = max(np.searchsorted(t, tm + bit / 4), lo + 1)
        in_burst = any(s <= tm < s + w for s, w, _ in info["bursts"])
        if in_burst:
            space_mid.append(np.min(vout[lo:hi]))
        else:
            mark_mid.append(np.max(vout[lo:hi]))
    idle = vout[(t > 5e-6) & (t < info["t_lead"] - 5e-6)]
    tail = vout[t > info["t_end"] + 20e-6]

    # atrasos de subida/descida (cruzamento dos limiares típicos)
    rise_d, fall_d = [], []
    def first_cross(i, above, th, limit):
        seg = vout[i:limit]
        hit = seg > th if above else seg < th
        return i + int(np.argmax(hit)) if hit.any() else None

    for s, w, _ in info["bursts"]:
        i0 = np.searchsorted(t, s)
        i1 = np.searchsorted(t, s + w)
        k = first_cross(i0, True, TH_RISE_TYP, i1)
        if k is not None:
            rise_d.append(t[k] - s)
        k = first_cross(i1, False, TH_FALL_TYP, np.searchsorted(t, s + w + bit))
        if k is not None:
            fall_d.append(t[k] - (s + w))

    r = dict(
        tag=info["tag"], baud=baud,
        sent=text_bytes.decode("latin1"), got=got.decode("latin1"),
        ok=got == text_bytes and not errs and not truncated,
        truncated=bool(truncated),
        errors=[(e, round(x * 1e6, 1)) for e, x in errs],
        v_space_min=float(np.min(space_mid)) if space_mid else None,
        v_mark_max=float(np.max(mark_mid)) if mark_mid else None,
        v_idle_max=float(max(np.max(idle) if len(idle) else 0,
                             np.max(tail) if len(tail) else 0)),
        rise_us=float(np.mean(rise_d) * 1e6) if rise_d else None,
        fall_us=float(np.mean(fall_d) * 1e6) if fall_d else None,
    )
    if r["rise_us"] is not None and r["fall_us"] is not None:
        r["suggested_trim_ns"] = int(round((r["fall_us"] - r["rise_us"]) * 1e3))
    r["worst_case_ok"] = bool(
        r["v_space_min"] is not None and r["v_space_min"] > TH_SPACE_WORST and
        (r["v_mark_max"] or 0) < TH_MARK_WORST and r["v_idle_max"] < TH_MARK_WORST
        and not truncated)

    if plot:
        plot_result(t, vout, venv, info, r, bit)
    return r


def plot_result(t, vout, venv, info, r, bit):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(12, 6.5), constrained_layout=True)
    for ax, (a, b) in zip(axes, [(0, t[-1]), (info["t_lead"] - bit, info["t_lead"] + 3 * bit)]):
        m = (t >= a) & (t <= b)
        ax.plot(t[m] * 1e3, venv[m] * 0.5 - 0.8, color="#888", lw=0.8,
                label="rajada Ethernet (envelope)")
        ax.plot(t[m] * 1e3, vout[m], color="#1f6feb", lw=1.2, label="saída do dongle → RxD")
        for y, c, lbl in [(TH_SPACE_WORST, "#d1242f", "2,4 V (space garantido)"),
                          (TH_RISE_TYP, "#bf8700", "1,5 V (típico)"),
                          (TH_MARK_WORST, "#1a7f37", "0,6 V (mark garantido)")]:
            ax.axhline(y, color=c, ls="--", lw=0.8, label=lbl)
        for tn in info["nlp"]:
            if a <= tn <= b:
                ax.axvline(tn * 1e3, color="#8250df", lw=0.6, alpha=0.6)
        ax.set_xlabel("tempo (ms)")
        ax.set_ylabel("V")
        ax.grid(alpha=0.3)
    axes[0].set_title(
        f"{r['tag']}: enviado {r['sent']!r} → decodificado {r['got']!r} "
        f"({'OK' if r['ok'] else 'FALHA'})")
    axes[1].set_title("zoom no primeiro caractere (linhas roxas = NLP)")
    axes[0].legend(loc="upper right", fontsize=7, ncol=3)
    fig.savefig(os.path.join(OUT, r["tag"] + ".png"), dpi=110)
    plt.close(fig)


def one(variant, baud, text, vline, trim_ns, nlp_mode="worst", plot=True,
        params=None, tag=None):
    tb = text.encode().decode("unicode_escape").encode("latin1")
    lead = max(0.2e-3, 2.0 / baud)
    info = build(variant, baud, text, vline, trim_ns, lead, nlp_mode, params, tag)
    run_spice(info["cir"])
    return analyze(info, baud, tb, plot)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variant", default="doubler", choices=["halfwave", "doubler", "stepup"])
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--text", default="U\\r")
    ap.add_argument("--vline", type=float, default=2.5, help="pico 10BASE-T em 100 ohm")
    ap.add_argument("--trim", type=int, default=0)
    ap.add_argument("--all", action="store_true", help="matriz completa de casos")
    ap.add_argument("--set", action="append", default=[], metavar="NOME=VALOR",
                    help="sobrescreve um componente (RIN, C1, RBL, R2, C2, RLOAD)")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(NICUART):
        raise SystemExit("compile antes: make")

    # (variante, baud, texto, pico da linha, trim_ns). 2,2 V e 2,8 V são os
    # limites da 802.3 para 10BASE-T; 3800 ns é o trim que a própria
    # simulação sugere para o dongle stepup (descida ~5,1 us - subida ~1,3 us).
    cases = []
    if a.all:
        for variant in ("halfwave", "doubler", "stepup"):
            for vline in (2.2, 2.8):
                cases.append((variant, 9600, "U\\r", vline, 0))
        cases.append(("stepup", 9600, "show ver\\r", 2.2, 3800))
        cases.append(("stepup", 19200, "U\\r", 2.2, 3800))
        cases.append(("stepup", 19200, "U\\r", 2.8, 3800))
        cases.append(("stepup", 1200, "\\x00U", 2.2, 3800))  # quadros divididos por IFG
    else:
        cases.append((a.variant, a.baud, a.text, a.vline, a.trim))

    results = []
    params = dict(kv.split("=", 1) for kv in a.set)
    for variant, baud, text, vline, trim in cases:
        r = one(variant, baud, text, vline, trim, params=params)
        r["trim_ns"] = trim
        results.append(r)
        fmt = lambda x, u: "  -  " if x is None else f"{x:.2f}{u}"
        print(f"{r['tag']:<29} {'OK   ' if r['ok'] else 'FALHA'} "
              f"enviado={r['sent']!r:<13} lido={r['got']!r:<13} "
              f"space_min={fmt(r['v_space_min'], 'V')} mark_max={fmt(r['v_mark_max'], 'V')} "
              f"idle_max={fmt(r['v_idle_max'], 'V')} subida={fmt(r['rise_us'], 'us')} "
              f"descida={fmt(r['fall_us'], 'us')} pior_caso={'OK' if r['worst_case_ok'] else 'não'}"
              + (" SIMULAÇÃO TRUNCADA" if r["truncated"] else "")
              + (f" erros={r['errors']}" if r['errors'] else ""))
    with open(os.path.join(OUT, "summary.json"), "w") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    # o meia-onda é o controle negativo: espera-se que falhe em 2,2 V
    required = [r for r in results if not r["tag"].startswith("halfwave")]
    return 0 if all(r["ok"] for r in required) else 1


if __name__ == "__main__":
    sys.exit(main())
