/*
 * encoder.h - converte bytes UART (8N1) em rajadas de quadros Ethernet 10BASE-T.
 *
 * Convenção: bit "space" (0, start bit) = linha com rajada Ethernet (o dongle
 * retifica e gera tensão positiva); bit "mark" (1, idle, stop) = linha em
 * silêncio (dongle em ~0 V, lido como mark pela maioria dos receptores RS232).
 *
 * O encoder é puro (sem I/O): só calcula quando cada quadro deve começar e
 * qual tamanho ele precisa ter para ocupar o fio pelo tempo do(s) bit(s).
 */
#ifndef NICUART_ENCODER_H
#define NICUART_ENCODER_H

#include <stddef.h>
#include <stdint.h>

/* 10BASE-T: 1 byte no fio = 8 bits x 100 ns */
#define NU_BYTE_NS_10M       800u
/* preâmbulo (7) + SFD (1) */
#define NU_PREAMBLE_BYTES    8u
/* inter-frame gap mínimo: 96 bit times = 12 bytes */
#define NU_IFG_BYTES         12u
/* tamanhos de quadro Ethernet incluindo FCS, sem preâmbulo */
#define NU_MIN_FRAME         64u
#define NU_MAX_FRAME         1518u

typedef struct {
	uint32_t baud;        /* 1200..19200 na prática */
	uint32_t data_bits;   /* 5..8 (normalmente 8) */
	uint32_t stop_bits;   /* 1 ou 2 */
	uint32_t trim_ns;     /* encurta cada rajada para compensar o atraso de
	                         descarga do detector (ver sim/) */
	double   max_stretch; /* quanto um bit "space" pode ser alongado por
	                         causa do quadro mínimo (0.15 = 15%) */
	uint32_t byte_ns;     /* NU_BYTE_NS_10M */
	uint32_t min_frame;   /* NU_MIN_FRAME */
	uint32_t max_frame;   /* NU_MAX_FRAME */
} nu_cfg;

typedef struct {
	uint64_t start_ns;    /* instante em que o preâmbulo deve começar */
	uint32_t frame_len;   /* bytes do quadro incluindo FCS (sem preâmbulo) */
	uint32_t wire_ns;     /* duração real no fio (preâmbulo + quadro) */
} nu_burst;

enum {
	NU_OK = 0,
	NU_ERR_ARG = -1,      /* configuração inválida */
	NU_ERR_SPACE = -2,    /* out[] pequeno demais */
	NU_ERR_SHORT = -3,    /* bit curto demais para o quadro mínimo (baud alto) */
	NU_ERR_LONG = -4,     /* bit longo demais para o quadro máximo (baud < ~820) */
};

/* Preenche cfg com padrões para o baud dado (8N1, 10 Mbps). */
void nu_cfg_default(nu_cfg *cfg, uint32_t baud);

/* Instante (ns, relativo ao início) da borda do bit de índice idx. */
uint64_t nu_bit_edge_ns(const nu_cfg *cfg, uint64_t idx);

/* Bits por caractere (start + dados + stop). */
uint32_t nu_bits_per_char(const nu_cfg *cfg);

/*
 * Codifica data[0..n) começando em t0_ns. Caracteres são enviados colados
 * (sem idle extra além dos stop bits). Retorna o número de rajadas escritas
 * em out (>= 0) ou um NU_ERR_*. Se t_end_ns != NULL, recebe o instante em que
 * o último stop bit termina.
 */
int nu_encode(const nu_cfg *cfg, const uint8_t *data, size_t n, uint64_t t0_ns,
	      nu_burst *out, size_t out_cap, uint64_t *t_end_ns);

/* Pior caso de rajadas por caractere (para dimensionar out[]). */
size_t nu_max_bursts_per_char(const nu_cfg *cfg);

const char *nu_strerror(int err);

#endif
