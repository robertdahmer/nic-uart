#!/usr/bin/env python3
"""
Teste de integração (precisa de root/CAP_NET_RAW): envia texto com
`nicuart send -i lo`, captura os quadros EtherType 0x88B5 no loopback e
confere que

  1. o número e o tamanho dos quadros batem com `nicuart encode`;
  2. o intervalo entre quadros segue o agendado (com tolerância folgada,
     porque aqui é um container/VM sem tempo real; o número medido é
     informativo, a medição séria é `nicuart jitter --hw` no i210);
  3. o modo pty entrega o que é escrito em /dev/ttyNIC0 (aqui num caminho
     temporário).
"""
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NICUART = os.path.join(ROOT, "nicuart")
ETH_P = 0x88B5


def capture(stop, frames):
    s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETH_P))
    s.bind(("lo", 0))
    s.settimeout(0.05)
    while not stop.is_set():
        try:
            data, addr = s.recvfrom(4096)
        except socket.timeout:
            continue
        # no lo o quadro pode aparecer na saída e na entrada; no modo spin o
        # nicuart usa PACKET_QDISC_BYPASS, que não passa pelos taps de saída,
        # então contamos só a cópia de entrada (broadcast)
        if addr[2] == socket.PACKET_OUTGOING:
            continue
        frames.append((time.monotonic_ns(), len(data)))
    s.close()


def encode(text, baud):
    out = subprocess.run([NICUART, "encode", "-b", str(baud), text],
                         capture_output=True, text=True, check=True).stdout
    rows = [l for l in out.splitlines() if l and l[0].isdigit()]
    return [tuple(int(x) for x in r.split(",")) for r in rows]


def run_case(text, baud, cmd):
    frames, stop = [], threading.Event()
    th = threading.Thread(target=capture, args=(stop, frames))
    th.start()
    time.sleep(0.1)
    cmd()
    time.sleep(0.2)
    stop.set()
    th.join()
    want = encode(text, baud)
    fails = []
    if len(frames) != len(want):
        fails.append(f"quadros: esperado {len(want)}, capturado {len(frames)}")
    else:
        for i, ((_, got_len), (_, flen, _)) in enumerate(zip(frames, want)):
            if got_len != flen - 4:  # sem FCS no loopback
                fails.append(f"quadro {i}: {got_len} B, esperado {flen - 4}")
        errs = []
        for i in range(1, len(want)):
            sched = want[i][0] - want[i - 1][0]
            meas = frames[i][0] - frames[i - 1][0]
            errs.append(abs(meas - sched))
        if errs:
            errs.sort()
            p50 = errs[len(errs) // 2] / 1e3
            mx = errs[-1] / 1e3
            print(f"    intervalo entre quadros: erro mediano {p50:.1f} us, máx {mx:.1f} us "
                  f"(container sem RT; só informativo)")
            if p50 > 1000:
                fails.append(f"erro mediano absurdo ({p50:.0f} us): agendamento quebrado")
    return fails


def main():
    if os.geteuid() != 0:
        print("pulado: precisa de root")
        return 0
    if not os.path.exists(NICUART):
        print("compile antes: make")
        return 1
    total = []

    text = "show ver\\r"
    print("[1] send no lo, 9600 bps")
    f = run_case(text, 9600, lambda: subprocess.run(
        [NICUART, "send", "-i", "lo", "-b", "9600", text], check=True,
        capture_output=True))
    total += f

    print("[2] send no lo, 1200 bps com quadros divididos")
    f = run_case("\\x00", 1200, lambda: subprocess.run(
        [NICUART, "send", "-i", "lo", "-b", "1200", "\\x00"], check=True,
        capture_output=True))
    total += f

    print("[3] modo pty")
    tmp = tempfile.mkdtemp()
    link = os.path.join(tmp, "ttyNIC0")
    proc = subprocess.Popen([NICUART, "pty", "-i", "lo", "--link", link],
                            stderr=subprocess.PIPE)
    for _ in range(50):
        if os.path.islink(link):
            break
        time.sleep(0.05)

    def write_pty():
        fd = os.open(link, os.O_WRONLY | os.O_NOCTTY)
        os.write(fd, b"show ver\r")
        os.close(fd)
        time.sleep(0.05)

    f = run_case(text, 9600, write_pty)
    proc.terminate()
    proc.wait(timeout=5)
    if os.path.islink(link):
        f.append("symlink não foi removido ao sair")
    total += f

    for x in total:
        print("  FALHOU:", x)
    print("test_loopback:", "todos os testes passaram" if not total else f"{len(total)} falha(s)")
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
