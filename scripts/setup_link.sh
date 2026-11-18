#!/usr/bin/env bash
# Fase 1 (Gate 1): prepara a interface para o NIC-UART e diz se ela transmite.
#
#   sudo scripts/setup_link.sh enp3s0
#
# - força 10BASE-T full duplex sem autonegociação (em 10BASE-T a linha fica
#   em SILÊNCIO entre quadros; em 100BASE-TX nunca fica, e o truque não serve)
# - desliga EEE e offloads irrelevantes
# - mostra driver, link e capacidade de timestamp/LaunchTime
#
# Sem link partner a maioria das placas não sobe link. Use o plugue de
# loopback passivo (pinos 1->3 e 2->6, ver hardware/dongle.md): a placa passa
# a ver os próprios NLPs e sobe link em 10 Mb/s.
set -euo pipefail

IF="${1:-}"
if [[ -z "$IF" ]]; then
	echo "uso: sudo $0 INTERFACE" >&2
	exit 1
fi
if [[ $EUID -ne 0 ]]; then
	echo "precisa de root" >&2
	exit 1
fi
command -v ethtool >/dev/null || { echo "instale o ethtool" >&2; exit 1; }

echo "== driver"
ethtool -i "$IF" | sed -n '1,3p'
DRIVER=$(ethtool -i "$IF" | awk '/^driver:/{print $2}')
case "$DRIVER" in
	igb|igc) echo "   ok: $DRIVER tem LaunchTime (etf offload) e timestamp de hardware" ;;
	e1000e)  echo "   aviso: e1000e não tem LaunchTime; use -m spin" ;;
	*)       echo "   aviso: driver $DRIVER não testado; use -m spin primeiro" ;;
esac

echo "== forçando 10BASE-T full duplex"
ip link set "$IF" up
ethtool -s "$IF" speed 10 duplex full autoneg off
ethtool --set-eee "$IF" eee off 2>/dev/null || true
# o payload é irrelevante; desligar offloads evita que o driver junte quadros
ethtool -K "$IF" tso off gso off gro off 2>/dev/null || true

echo "== aguardando link (5 s)"
for _ in $(seq 1 10); do
	if [[ "$(cat /sys/class/net/$IF/carrier 2>/dev/null || echo 0)" == "1" ]]; then
		break
	fi
	sleep 0.5
done
ethtool "$IF" | grep -E 'Speed|Duplex|Auto-negotiation|Link detected|MDI-X' || true

if [[ "$(cat /sys/class/net/$IF/carrier 2>/dev/null || echo 0)" != "1" ]]; then
	cat <<MSG

!! sem link. Opções, em ordem:
   1. encaixe o plugue de loopback (1->3, 2->6) e rode de novo;
   2. confira com o osciloscópio se há NLPs (pulsos de ~100 ns a cada 16 ms)
      nos pinos 1/2. Se o PHY estiver em MDI-X, eles estarão nos pinos 3/6:
      mova o dongle para esse par;
   3. plano B: patch no driver para forçar link (CTRL.SLU no igb), ver
      docs/feasibility.md, fase 1.
MSG
	exit 2
fi

echo "== timestamp de hardware"
ethtool -T "$IF" | grep -E 'hardware-transmit|PTP Hardware Clock' || true

cat <<MSG

Gate 1: com o osciloscópio nos pinos 1/2 do RJ45 (ou do loopback), rode
   sudo ./nicuart send -i $IF -b 9600 'U'
e confira: 5 rajadas de ~104 us separadas por ~104 us de silêncio.
Depois meça a precisão:
   sudo ./nicuart jitter -i $IF --rt --cpu 2 --hw
MSG
