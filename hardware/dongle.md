# Dongle passivo NIC-UART (variante `stepup`)

Converte as rajadas 10BASE-T do par TX da placa de rede em um nível 0 V / +2,6–3,4 V
na entrada RxD do console. Sem alimentação, sem microcontrolador, sem USB.

> Só faz **TX** (PC → equipamento). Ver `docs/feasibility.md` para o porquê.

## Esquema

```
 Par TX da placa (via plugue de loopback)                 Para o console do equipamento
                                                           (RJ45 Cisco, visto do equipamento)

 pino 1 (TX+) ──RIN 47R──┐      ┌──|>|──┬───R2 1k───┬──────────────── pino 6  RxD (entra no equipamento)
                         │  T1  │  D1   │           │
                        LT1 ‖  LT2 BAT54│           │
                     5 esp. ‖ 10 esp.  C1 1n  RBL  C2 1n
                         │      │       │    4k7    │
 pino 2 (TX−) ───────────┴──────┴───────┴─────┴─────┴──────────────── pinos 4/5  GND
```

- **T1**: transformador elevador 1:2 num toroide de ferrite. Com FT37-43
  (AL ≈ 350 nH/esp²): 5 espiras no primário (~9 µH) e 10 no secundário
  (~35 µH), em fio esmaltado de 0,2–0,3 mm. Com FT37-61 (AL ≈ 55 nH/esp²):
  13 e 26 espiras. O simulador usa 10 µH / 40 µH, k = 0,98.
- **D1**: Schottky de sinal (BAT54, BAT43, 1N5711). Diodo de silício comum
  perde 0,4 V a mais e come quase toda a margem.
- **C1** (reservatório) e **RBL** (descarga): definem a descida, ~4,5–5,7 µs.
- **R2/C2**: filtro passa-baixa. Mata os NLPs (pulsos de 100 ns a cada 16 ms)
  e o TP_IDL do fim de cada quadro. Na simulação eles ficam em < 0,4 V.
- A entrada do receptor RS232 (3–7 kΩ) é a carga. O simulador usa 5 kΩ.

As variantes `halfwave` (sem T1) e `doubler` (dobrador Greinacher) estão em
`sim/` para comparação. Nenhuma das duas fecha a margem de pior caso (ver abaixo).

## Plugue de loopback

Sem um link partner a placa não sobe link e não transmite. A solução passiva
é fechar o TX no próprio RX, para a placa ver os próprios NLPs:

```
 RJ45 (cabo vindo do PC)
   pino 1 (TX+) ──┬──────── pino 3 (RX+)
                  └── dongle (pino 1 acima)
   pino 2 (TX−) ──┬──────── pino 6 (RX−)
                  └── dongle (pino 2 acima)
```

A terminação de 100 Ω do RX da própria placa carrega o par TX (o simulador
inclui isso). Se o PHY entrar em MDI-X, o TX aparece nos pinos 3/6. Confira
com o osciloscópio em qual par estão as rajadas e ligue o dongle nesse par.

## Lista de peças (por dongle)

| Ref.  | Peça                          | Obs.                                  |
|-------|-------------------------------|---------------------------------------|
| J1    | conector RJ45 fêmea (do PC)   | com o loopback 1→3, 2→6 na placa      |
| J2    | cabo/conector RJ45 (console)  | só pinos 4, 5 (GND) e 6 (RxD)          |
| T1    | toroide FT37-43 + fio 0,25 mm | 5 : 10 espiras                        |
| D1    | BAT54                         | ou BAT43 / 1N5711                     |
| RIN   | 47 Ω                          | limita corrente / protege             |
| R2    | 1 kΩ                          |                                       |
| RBL   | 4,7 kΩ                        |                                       |
| C1,C2 | 1 nF cerâmico (C0G)           |                                       |

## Ligação na bancada (fase 4)

```
 PC (i210) ──cabo── [J1: loopback + dongle] ──J2──> console do equipamento
                                                       pino 6 RxD  <- saída do dongle
                                                       pinos 4/5   <- GND
 USB-serial de referência (só para ver a resposta):
   RX do adaptador <- pino 3 TxD do equipamento ; GND <- pino 4/5
```

**Cuidados**
- Use uma placa PCIe de sacrifício (i210), não a placa onboard do notebook.
- **Não** ligue o TxD do equipamento (±5 a ±12 V) na placa de rede nas
  fases 1–4. Na fase 5 (investigação de RX), use um resistor de 10 kΩ em série.
- Confira a pinagem no guia de hardware do seu equipamento. A de cima é a
  porta de console RJ45 padrão da Cisco (DTE: 3 = TxD, 6 = RxD).
