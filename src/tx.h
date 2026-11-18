/*
 * tx.h - envia as rajadas calculadas pelo encoder como quadros Ethernet crus.
 *
 * Dois modos de temporização:
 *   NU_MODE_SPIN   - espera ativa em CLOCK_MONOTONIC e send() na hora certa.
 *                    Funciona em qualquer NIC; jitter depende do kernel/CPU.
 *   NU_MODE_TXTIME - SO_TXTIME + qdisc etf (com offload = LaunchTime do i210/
 *                    i225/i226). O NIC dispara o quadro no instante exato.
 */
#ifndef NICUART_TX_H
#define NICUART_TX_H

#include <stddef.h>
#include <stdint.h>
#include "encoder.h"

#define NU_ETHERTYPE 0x88B5 /* IEEE 802 "local experimental" */
#define NU_TXTIME_PRIO 3    /* SO_PRIORITY usado no modo txtime */

typedef enum { NU_MODE_SPIN = 0, NU_MODE_TXTIME = 1 } nu_mode;

typedef struct {
	const char *ifname;
	nu_mode mode;
	int cpu;            /* -1 = não fixar */
	int rt;             /* SCHED_FIFO + mlockall */
	int timestamps;     /* 0 = não, 1 = software, 2 = hardware */
	uint64_t lead_ns;   /* antecedência entre "agora" e a 1a rajada */
	uint64_t horizon_ns;/* txtime: quanto à frente enfileirar */
	int verbose;
} nu_tx_opts;

typedef struct {
	int fd;
	int ifindex;
	uint8_t mac[6];
	nu_tx_opts o;
	uint32_t next_id;   /* contador de SOF_TIMESTAMPING_OPT_ID */
	uint8_t frame[NU_MAX_FRAME];
	uint64_t sent, send_errors, txtime_drops;
} nu_tx;

typedef struct {
	uint32_t id;        /* índice do quadro desde a abertura */
	uint64_t ts_ns;     /* horário em que o quadro saiu (sw ou hw) */
} nu_txts;

int nu_tx_open(nu_tx *tx, const nu_tx_opts *o);
void nu_tx_close(nu_tx *tx);

/* Relógio usado para agendar (MONOTONIC no modo spin, TAI no txtime). */
uint64_t nu_tx_now(const nu_tx *tx);

/*
 * Envia as rajadas b[0..n), cujos start_ns são absolutos no relógio de
 * nu_tx_now(). Bloqueia até a última ser entregue ao kernel. Retorna 0 ou
 * -errno. first_id (opcional) recebe o id de timestamp do primeiro quadro.
 */
int nu_tx_send(nu_tx *tx, const nu_burst *b, size_t n, uint32_t *first_id);

/* Lê timestamps/erros pendentes da fila de erros (não bloqueia).
 * Retorna quantos timestamps foram escritos em ts. */
size_t nu_tx_drain(nu_tx *tx, nu_txts *ts, size_t cap);

#endif
