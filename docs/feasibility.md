# NIC-UART: viabilidade de falar RS232 pela placa de rede

Estado: **simulação concluída, bancada pendente.**

| Gate | O que prova | Onde | Resultado |
|------|-------------|------|-----------|
| 0 | o encoder gera exatamente os bits UART pedidos | container (`make test`) | ✅ roundtrip dos 256 bytes, 1200–19200 bps |
| 0 | o programa envia os quadros agendados (send/pty) | container, `lo` (`make test-lo`) | ✅ número, tamanho e ordem corretos |
| 0 | um dongle passivo recupera um RS232 legível | ngspice (`make sim`) | ✅ variante `stepup`, 9600 bps, pior caso 802.3 e MAX3232 |
| 1 | a placa transmite sem link partner, com silêncio entre quadros | bancada | ⏳ |
| 1 | o jitter de envio cabe na tolerância do UART | bancada (`nicuart jitter --hw`) | ⏳ |
| 2 | um equipamento real recebe e executa um comando | bancada | ⏳ |
| RX | dá para receber pelo NIC sem eletrônica ativa | bancada (`nicuart rxprobe`) | ❌ esperado inviável (ver §5) |

## 1. Por que "só um driver" não resolve

| | Porta Ethernet | RS232 |
|---|---|---|
| Sinal | diferencial, por par | single-ended, contra GND |
| Níveis | ±2,2–2,8 V (10BASE-T) | ±3 a ±15 V |
| Acoplamento | transformador (magnetics) | DC |
| Quem gera | o PHY, em hardware | o UART |

- Os magnetics têm no mínimo 350 µH (802.3). Contra ~100 Ω, a constante de
  tempo fica em ~3,5 µs, e um bit a 9600 bps dura 104 µs. **Um nível DC não
  atravessa.**
- O driver fala com o MAC. A forma de onda (Manchester, NLP, TP_IDL) é gerada
  pelo PHY. Não existe registrador para "colocar +5 V na linha".
- O par é isolado, sem GND comum com o equipamento.

**O que o software controla:** em que instante um quadro começa e quanto ele
dura. Esse é o canal explorado aqui.

## 2. A ideia que funciona (no papel)

- Em **10BASE-T** a linha fica em silêncio entre quadros. Só passam NLPs de
  100 ns a cada 16 ms. Em 100BASE-TX a linha nunca fica quieta (IDLE
  embaralhado), por isso o truque exige forçar 10 Mb/s.
- Bit **space** (0, start bit) vira uma rajada de quadro com a duração do bit.
  Bit **mark** (1, idle, stop) vira silêncio.
- Um **detector de envelope passivo** retifica a rajada e entrega 0 V / +2,6 V.
  A maioria dos receptores RS232 lê 0 V como mark, porque tem pulldown interno
  e limiares de 0,6 a 2,4 V.

### Contas de tempo (10 Mb/s, 1 byte = 800 ns)

| Baud | 1 bit | bytes no fio | quadro (sem preâmbulo) | situação |
|------|-------|--------------|------------------------|----------|
| 1200 | 833 µs | 1042 | 1034 | ok. Trechos com mais de 1 bit são divididos **na borda do bit**, com IFG de 9,6 µs |
| 9600 | 104 µs | 130 | 122 | **ok: é o padrão do console Cisco** |
| 19200 | 52 µs | 65 | 57 < 64 | o quadro mínimo alonga o bit em 10,6%. Ver §3 |
| 38400 | 26 µs | 33 | — | **impossível**: o bit é mais curto que o menor quadro Ethernet |
| < ~820 | — | > 1526 | — | impossível: o bit não cabe num quadro de 1518 B |

A quantização é de 800 ns (< 0,8% do bit a 9600). `src/encoder.c` funde bits
space consecutivos num só quadro e respeita os limites de 64/1518 B e o IFG.

## 3. Simulação do dongle (`make sim`)

Modelo: driver 10BASE-T com Rs = 100 Ω, magnetics 1:1 de 350 µH, terminação
de 100 Ω do loopback, dongle e carga de 5 kΩ do receptor. A portadora é uma
onda quadrada de 10 MHz, com NLP e TP_IDL incluídos. **Os NLPs são postos no
pior lugar possível:** no idle antes do texto (testa falso start bit) e no
meio de um bit mark (testa bit trocado). A saída passa por um receptor com os
limiares do MAX3232 e por um UART 16x em software.

Limiares de **pior caso**: space > 2,4 V e mark < 0,6 V, avaliados numa janela
de ±¼ de bit em torno do meio do bit.

| Caso | Pico da linha | Lido | space mín. | mark máx. | Pior caso |
|------|---------------|------|------------|-----------|-----------|
| halfwave 9600 | 2,2 V | ❌ nada | 1,25 V | 0,31 V | ❌ |
| halfwave 9600 | 2,8 V | ✅ | 1,62 V | 0,41 V | ❌ |
| doubler 9600 | 2,2 V | ✅ | 2,40 V | 0,55 V | ❌ (no limite) |
| doubler 9600 | 2,8 V | ✅ | 3,13 V | 0,71 V | ❌ |
| **stepup 9600** | 2,2 V | ✅ | **2,65 V** | **0,29 V** | ✅ |
| **stepup 9600** | 2,8 V | ✅ | **3,40 V** | **0,38 V** | ✅ |
| stepup 9600 "show ver\r" (trim 3,8 µs) | 2,2 V | ✅ | 2,65 V | 0,29 V | ✅ |
| stepup 19200 | 2,2 V | ✅ | 2,65 V | 0,63 V | ❌ (marginal) |
| stepup 19200 | 2,8 V | ✅ | 3,40 V | 0,81 V | ❌ (marginal) |
| stepup 1200 "\x00U" (quadros divididos) | 2,2 V | ✅ | 2,65 V | 0,29 V | ✅ |

Conclusões:
- **Meia onda pura não basta.** A queda do Schottky somada ao divisor com o
  receptor deixa o nível abaixo até do limiar típico.
- **O dobrador lê com limiares típicos, mas sem margem.** O transformador
  elevador 1:2 é o que fecha o pior caso, ao custo de um toroide de centavos.
- **9600 bps é robusto. 19200 é marginal:** o quadro mínimo e a descida do
  RC invadem o bit seguinte.
- Subida de ~1,3 µs e descida de ~5 µs. O `--trim 3800` encurta cada rajada
  para centralizar os bits (sugerido pela própria simulação).

Gráficos em `sim/out/*.png` e números em `sim/out/summary.json`. Para
experimentar outros valores: `python3 sim/run_sim.py --variant stepup --set C1=470p`.

**Limites do modelo:** portadora idealizada (Manchester real tem conteúdo em
5 e 10 MHz), diodo genérico, magnetics sem perdas no núcleo e receptor modelado
só por limiares. Os números finais são os da bancada.

## 4. Roteiro de bancada

Material: Intel i210 PCIe (driver `igb`), osciloscópio, dongle + loopback
(`hardware/dongle.md`), um equipamento com console RS232 e um USB-serial
**só como instrumento** para ver a resposta.

1. **Gate 1, link e silêncio**
   ```
   make && sudo scripts/setup_link.sh enp3s0
   sudo ./nicuart send -i enp3s0 -b 9600 'U'
   ```
   No osciloscópio, nos pinos 1/2: 5 rajadas de ~104 µs separadas por
   ~104 µs de silêncio. Se não houver link, use o loopback. Se mesmo assim
   não houver, o plano B é um patch no `igb` que seta `CTRL.SLU`. Se a placa
   nunca transmitir sem partner, **o projeto para aqui** (registrar o resultado).
2. **Gate 1, jitter**
   ```
   sudo ./nicuart jitter -i enp3s0 --rt --cpu 2 --hw
   ```
   Critério: todos os erros abaixo de ¼ de bit (26 µs a 9600). Se falhar:
   `isolcpus=2` na linha do kernel, kernel PREEMPT_RT, ou o modo B
   (`sudo scripts/setup_etf.sh enp3s0` + `-m txtime`).
   **A verificar:** se o LaunchTime do i210 funciona a 10 Mb/s.
3. **Dongle**: com o osciloscópio na saída do dongle (carregada com o MAX3232
   ou 5 kΩ), conferir níveis e bordas contra a tabela do §3. Ajustar o `--trim`.
4. **Gate 2, equipamento real**
   ```
   sudo ./nicuart send -i enp3s0 --trim 3800 '\r'
   sudo ./nicuart send -i enp3s0 --trim 3800 'show version\r'
   ```
   A resposta aparece no USB-serial de referência. Depois, a taxa de erro:
   enviar 1000 caracteres conhecidos (ex.: num `banner` ou no prompt) e contar
   os errados.
5. **Uso "de verdade"** (às cegas):
   ```
   sudo ./nicuart pty -i enp3s0 --trim 3800 --echo
   picocom -b 9600 /dev/ttyNIC0
   sudo ./nicuart send -i enp3s0 --trim 3800 -f config.txt --line-delay 300
   ```

## 5. RX (equipamento → PC): por que se espera que não dê

- O PHY só entrega ao MAC **quadros Manchester válidos**, com preâmbulo, SFD
  e FCS. Um RS232 entrando nos pinos de RX vira, depois dos magnetics, picos
  de ~3,5 µs a cada borda. Isso não é quadro, e não chega ao driver.
- O que sobra são **registradores de status** do PHY, via MDIO (energy detect,
  polaridade, contadores de falso carrier). Problemas:
  - uma leitura MDIO via ioctl custa dezenas de µs, e a 9600 bps seria preciso
    amostrar a cada ≤ 6,5 µs (16x) ou, no limite, ~35 µs (3x);
  - energy detect tem filtro/temporização própria (de ms) e **não tem
    polaridade**: só diz "teve borda", não se foi 0→1 ou 1→0;
  - contadores contam eventos, não dizem quando aconteceram.
- `nicuart rxprobe -i IF --reg R --mask M` mede a taxa de leitura real e
  registra cada mudança, para fechar essa questão com dados (fase 5, com
  resistor de 10 kΩ em série no TxD do equipamento).

**Mínimo de eletrônica ativa para ter RX:** algo do lado do equipamento que
gere quadros Ethernet válidos, ou seja, um microcontrolador com MAC/PHY
(W5500, ESP32 com PHY, etc.). Nesse ponto o projeto vira um adaptador de
console em rede (Ethernet/Wi-Fi + RFC 2217), que é o caminho prático se o
objetivo for console bidirecional sem USB.

## 6. Windows (depois)

O Windows não tem `AF_PACKET`, `SO_TXTIME` nem etf. Exigiria um driver de
filtro NDIS (ou KMDF específico do i210) para agendar quadros, mais o com0com
para expor a COM virtual. Só faz sentido se o Gate 2 passar no Linux.
