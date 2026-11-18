#!/usr/bin/env bash
# Modo B (-m txtime): o NIC dispara cada quadro no instante exato (LaunchTime).
# Testado conceitualmente para i210 (igb) e i225/i226 (igc).
#
#   sudo scripts/setup_etf.sh enp3s0
#
# Monta mqprio com a prioridade 3 (usada pelo nicuart, NU_TXTIME_PRIO) na fila
# 0, coloca o qdisc etf com offload nela e sincroniza o PHC da placa com
# CLOCK_TAI (o etf agenda em CLOCK_TAI; o LaunchTime compara com o PHC).
#
# A VERIFICAR NA BANCADA: o datasheet do i210 não deixa claro se o LaunchTime
# vale a 10 Mb/s. Se `nicuart jitter -m txtime --hw` mostrar erro alto ou o
# etf descartar quadros, volte para -m spin.
set -euo pipefail

IF="${1:-}"
DELTA_NS="${DELTA_NS:-300000}"   # antecedência que o etf exige (300 us)
if [[ -z "$IF" || $EUID -ne 0 ]]; then
	echo "uso: sudo $0 INTERFACE" >&2
	exit 1
fi
for c in tc phc2sys python3; do
	command -v "$c" >/dev/null || { echo "falta $c (iproute2 / linuxptp)" >&2; exit 1; }
done

echo "== mqprio: prio 3 -> TC0 -> fila 0"
tc qdisc replace dev "$IF" parent root handle 100 mqprio \
	num_tc 3 map 2 2 1 0 2 2 2 2 2 2 2 2 2 2 2 2 \
	queues 1@0 1@1 2@2 hw 0

echo "== etf com offload na fila 0 (delta ${DELTA_NS} ns)"
tc qdisc replace dev "$IF" parent 100:1 etf clockid CLOCK_TAI \
	delta "$DELTA_NS" offload
tc qdisc show dev "$IF"

# diferença atual entre CLOCK_TAI e CLOCK_REALTIME (0 se o kernel não tem o
# offset TAI configurado; 37 se o ptp4l/chrony já configurou)
TAI_OFF=$(python3 -c 'import time; print(round(time.clock_gettime(time.CLOCK_TAI) - time.time()))')
echo "== sincronizando PHC de $IF com CLOCK_TAI (offset TAI-UTC = ${TAI_OFF} s)"
echo "   (deixe rodando; Ctrl+C para parar)"
exec phc2sys -s CLOCK_REALTIME -c "$IF" -O "$TAI_OFF" -m
