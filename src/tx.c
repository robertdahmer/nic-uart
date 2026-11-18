#define _GNU_SOURCE
#include "tx.h"

#include <arpa/inet.h>
#include <errno.h>
#include <linux/errqueue.h>
#include <linux/if_packet.h>
#include <linux/net_tstamp.h>
#include <linux/sockios.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#ifndef SO_TXTIME
#define SO_TXTIME 61
#endif
#ifndef SCM_TXTIME
#define SCM_TXTIME SO_TXTIME
#endif
#ifndef SO_EE_ORIGIN_TXTIME
#define SO_EE_ORIGIN_TXTIME 6
#endif
#ifndef SOF_TXTIME_REPORT_ERRORS
#define SOF_TXTIME_REPORT_ERRORS (1 << 1)
#endif
#ifndef PACKET_QDISC_BYPASS
#define PACKET_QDISC_BYPASS 20
#endif

static uint64_t clock_ns(clockid_t id)
{
	struct timespec ts;
	clock_gettime(id, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

uint64_t nu_tx_now(const nu_tx *tx)
{
	return clock_ns(tx->o.mode == NU_MODE_TXTIME ? CLOCK_TAI : CLOCK_MONOTONIC);
}

static int enable_hw_timestamps(int fd, const char *ifname)
{
	struct hwtstamp_config cfg = { .tx_type = HWTSTAMP_TX_ON,
				       .rx_filter = HWTSTAMP_FILTER_NONE };
	struct ifreq ifr;
	memset(&ifr, 0, sizeof ifr);
	snprintf(ifr.ifr_name, sizeof ifr.ifr_name, "%s", ifname);
	ifr.ifr_data = (char *)&cfg;
	return ioctl(fd, SIOCSHWTSTAMP, &ifr);
}

int nu_tx_open(nu_tx *tx, const nu_tx_opts *o)
{
	memset(tx, 0, sizeof *tx);
	tx->o = *o;
	tx->fd = socket(AF_PACKET, SOCK_RAW, 0); /* protocolo 0: só envio */
	if (tx->fd < 0)
		return -errno;

	struct ifreq ifr;
	memset(&ifr, 0, sizeof ifr);
	snprintf(ifr.ifr_name, sizeof ifr.ifr_name, "%s", o->ifname);
	if (ioctl(tx->fd, SIOCGIFINDEX, &ifr) < 0)
		goto fail;
	tx->ifindex = ifr.ifr_ifindex;
	if (ioctl(tx->fd, SIOCGIFHWADDR, &ifr) < 0)
		goto fail;
	memcpy(tx->mac, ifr.ifr_hwaddr.sa_data, 6);

	struct sockaddr_ll sll = { .sll_family = AF_PACKET,
				   .sll_ifindex = tx->ifindex };
	if (bind(tx->fd, (struct sockaddr *)&sll, sizeof sll) < 0)
		goto fail;

	if (o->mode == NU_MODE_TXTIME) {
		struct { clockid_t clockid; uint32_t flags; } st = {
			CLOCK_TAI, SOF_TXTIME_REPORT_ERRORS };
		if (setsockopt(tx->fd, SOL_SOCKET, SO_TXTIME, &st, sizeof st) < 0)
			goto fail;
		/* prioridade 3 -> TC0 -> fila 0 no mqprio de scripts/setup_etf.sh
		 * (no i210 só as filas 0 e 1 têm LaunchTime) */
		int prio = NU_TXTIME_PRIO;
		if (setsockopt(tx->fd, SOL_SOCKET, SO_PRIORITY, &prio, sizeof prio) < 0)
			goto fail;
	} else {
		/* sem qdisc no caminho = menos latência e menos jitter */
		int one = 1;
		setsockopt(tx->fd, SOL_PACKET, PACKET_QDISC_BYPASS, &one, sizeof one);
	}

	if (o->timestamps) {
		unsigned flags = SOF_TIMESTAMPING_OPT_ID | SOF_TIMESTAMPING_OPT_TSONLY;
		if (o->timestamps == 2) {
			if (enable_hw_timestamps(tx->fd, o->ifname) < 0) {
				fprintf(stderr, "aviso: SIOCSHWTSTAMP falhou (%s); "
					"usando timestamps de software\n", strerror(errno));
				tx->o.timestamps = 1;
			}
		}
		if (tx->o.timestamps == 2)
			flags |= SOF_TIMESTAMPING_TX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE;
		else
			flags |= SOF_TIMESTAMPING_TX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE;
		if (setsockopt(tx->fd, SOL_SOCKET, SO_TIMESTAMPING, &flags, sizeof flags) < 0)
			goto fail;
		/* a fila de erros (onde chegam os timestamps) é limitada pelo
		 * rcvbuf; sem isso perdemos timestamps em rajadas longas */
		int rcv = 16 << 20;
		if (setsockopt(tx->fd, SOL_SOCKET, SO_RCVBUFFORCE, &rcv, sizeof rcv) < 0)
			setsockopt(tx->fd, SOL_SOCKET, SO_RCVBUF, &rcv, sizeof rcv);
	}

	if (o->cpu >= 0) {
		cpu_set_t set;
		CPU_ZERO(&set);
		CPU_SET(o->cpu, &set);
		if (sched_setaffinity(0, sizeof set, &set) < 0)
			fprintf(stderr, "aviso: sched_setaffinity: %s\n", strerror(errno));
	}
	if (o->rt) {
		struct sched_param sp = { .sched_priority = 80 };
		if (sched_setscheduler(0, SCHED_FIFO, &sp) < 0)
			fprintf(stderr, "aviso: SCHED_FIFO: %s\n", strerror(errno));
		if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0)
			fprintf(stderr, "aviso: mlockall: %s\n", strerror(errno));
	}

	/* cabeçalho fixo: broadcast, MAC da interface, EtherType experimental.
	 * O conteúdo do payload não importa para o dongle: em 10BASE-T todo
	 * byte vira Manchester com a mesma amplitude. */
	memset(tx->frame, 0, sizeof tx->frame);
	memset(tx->frame, 0xff, 6);
	memcpy(tx->frame + 6, tx->mac, 6);
	tx->frame[12] = NU_ETHERTYPE >> 8;
	tx->frame[13] = NU_ETHERTYPE & 0xff;
	memcpy(tx->frame + 14, "NICUART", 7);
	return 0;

fail:;
	int e = errno;
	close(tx->fd);
	tx->fd = -1;
	return -e;
}

void nu_tx_close(nu_tx *tx)
{
	if (tx->fd >= 0)
		close(tx->fd);
	tx->fd = -1;
}

static int send_one(nu_tx *tx, const nu_burst *b)
{
	/* o NIC acrescenta o FCS (4 bytes) */
	size_t len = b->frame_len - 4;
	struct iovec iov = { tx->frame, len };
	struct msghdr msg = { .msg_iov = &iov, .msg_iovlen = 1 };
	union {
		char buf[CMSG_SPACE(sizeof(uint64_t))];
		struct cmsghdr align;
	} u;

	if (tx->o.mode == NU_MODE_TXTIME) {
		memset(&u, 0, sizeof u);
		msg.msg_control = u.buf;
		msg.msg_controllen = sizeof u.buf;
		struct cmsghdr *c = CMSG_FIRSTHDR(&msg);
		c->cmsg_level = SOL_SOCKET;
		c->cmsg_type = SCM_TXTIME;
		c->cmsg_len = CMSG_LEN(sizeof(uint64_t));
		memcpy(CMSG_DATA(c), &b->start_ns, sizeof(uint64_t));
	}

	for (;;) {
		ssize_t r = sendmsg(tx->fd, &msg, 0);
		if (r == (ssize_t)len) {
			tx->sent++;
			tx->next_id++;
			return 0;
		}
		if (r < 0 && (errno == ENOBUFS || errno == EAGAIN))
			continue; /* fila cheia: tentar de novo (spin) */
		tx->send_errors++;
		return r < 0 ? -errno : -EIO;
	}
}

int nu_tx_send(nu_tx *tx, const nu_burst *b, size_t n, uint32_t *first_id)
{
	if (first_id)
		*first_id = tx->next_id;
	for (size_t i = 0; i < n; i++) {
		uint64_t target = b[i].start_ns;
		if (tx->o.mode == NU_MODE_TXTIME) {
			/* basta entregar ao etf com alguma antecedência */
			if (target > tx->o.horizon_ns) {
				uint64_t wake = target - tx->o.horizon_ns;
				while (nu_tx_now(tx) < wake) {
					struct timespec ts = { 0, 200000 };
					nanosleep(&ts, NULL);
				}
			}
		} else {
			/* espera ativa até o instante exato */
			while (nu_tx_now(tx) < target)
				;
		}
		int r = send_one(tx, &b[i]);
		if (r < 0)
			return r;
	}
	return 0;
}

size_t nu_tx_drain(nu_tx *tx, nu_txts *out, size_t cap)
{
	size_t n = 0;
	for (;;) {
		char ctrl[512];
		char data[64];
		struct iovec iov = { data, sizeof data };
		struct msghdr msg = { .msg_iov = &iov, .msg_iovlen = 1,
				      .msg_control = ctrl, .msg_controllen = sizeof ctrl };
		if (recvmsg(tx->fd, &msg, MSG_ERRQUEUE | MSG_DONTWAIT) < 0)
			break;

		uint64_t ts = 0;
		int have_ts = 0;
		struct sock_extended_err *ee = NULL;
		for (struct cmsghdr *c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c)) {
			if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPING) {
				struct timespec t[3];
				memcpy(t, CMSG_DATA(c), sizeof t);
				struct timespec *p = tx->o.timestamps == 2 ? &t[2] : &t[0];
				ts = (uint64_t)p->tv_sec * 1000000000ull + (uint64_t)p->tv_nsec;
				have_ts = ts != 0;
			} else if ((c->cmsg_level == SOL_PACKET &&
				    c->cmsg_type == PACKET_TX_TIMESTAMP) ||
				   (c->cmsg_level == SOL_IP && c->cmsg_type == IP_RECVERR)) {
				ee = (struct sock_extended_err *)CMSG_DATA(c);
			}
		}
		if (ee && ee->ee_origin == SO_EE_ORIGIN_TXTIME) {
			tx->txtime_drops++;
			if (tx->o.verbose)
				fprintf(stderr, "etf descartou quadro (código %u)\n", ee->ee_code);
			continue;
		}
		if (have_ts && n < cap) {
			out[n].id = ee ? ee->ee_data : UINT32_MAX;
			out[n].ts_ns = ts;
			n++;
		}
	}
	return n;
}
