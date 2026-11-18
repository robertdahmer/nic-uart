# nic-uart

Experimento: mandar RS232 para o console de um equipamento **pela porta RJ45
da placa de rede**, sem adaptador USB-serial.

Um driver sozinho não consegue, porque os transformadores da porta bloqueiam
DC. A ideia aqui é outra: forçar 10BASE-T, transformar cada bit "0" do UART
numa rajada de quadro Ethernet e cada bit "1" em silêncio, e usar um **dongle
passivo** (toroide 1:2 + Schottky + RC) para converter isso em nível RS232.

- ✅ Encoder e transmissor prontos e testados (`make test`, `make test-lo`).
- ✅ Simulação: o dongle `stepup` entrega um RS232 legível a 9600 bps, mesmo
  nos limites de pior caso da 802.3 e do MAX3232 (`make sim`).
- ⏳ Bancada: falta provar numa placa real (Intel i210) e num equipamento real.
- ❌ **Só TX** (PC → equipamento). Receber pelo NIC é, ao que tudo indica,
  inviável sem eletrônica ativa.

Detalhes, números e roteiro de bancada: [`docs/feasibility.md`](docs/feasibility.md).
Esquema e peças: [`hardware/dongle.md`](hardware/dongle.md).

## Compilar e testar

```bash
make            # ./nicuart
make test       # testes do encoder
sudo make test-lo   # integração no loopback (captura os quadros enviados)
make sim        # ngspice + UART em software; gráficos em sim/out/
```

`make sim` precisa de `ngspice`, `numpy` e `matplotlib`.

## Uso (Linux, root)

```bash
sudo scripts/setup_link.sh enp3s0                     # 10BASE-T forçado, checa link
sudo ./nicuart send -i enp3s0 --trim 3800 'show version\r'
sudo ./nicuart send -i enp3s0 --trim 3800 -f config.txt --line-delay 300
sudo ./nicuart pty  -i enp3s0 --trim 3800 --echo       # depois: picocom -b 9600 /dev/ttyNIC0
sudo ./nicuart jitter -i enp3s0 --rt --cpu 2 --hw      # mede a precisão do envio
sudo ./nicuart rxprobe -i enp3s0 --reg 1                # investigação de RX (registrador/bit: ver datasheet do PHY)
./nicuart encode -b 9600 'U'                            # só mostra as rajadas
```

## Estrutura

```
src/encoder.[ch]   bytes UART 8N1 -> rajadas (início, tamanho do quadro)
src/tx.[ch]        AF_PACKET; modos spin (espera ativa) e txtime (SO_TXTIME/etf)
src/nicuart.c      CLI: encode, send, pty, jitter, rxprobe
tests/             teste do encoder (C) e integração no loopback (Python)
sim/               netlists do dongle (halfwave, doubler, stepup) e run_sim.py
scripts/           setup_link.sh (Gate 1) e setup_etf.sh (LaunchTime)
hardware/          esquema, loopback, lista de peças, ligação
docs/              viabilidade, resultados e roteiro de bancada
```

## Segurança

Use uma placa de rede de sacrifício. Nunca ligue o TxD do equipamento
(±12 V) direto na placa de rede. Confira a pinagem do console no manual do
equipamento.
