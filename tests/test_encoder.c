#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/encoder.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { if (++fails > 20) break; \
	printf("FALHOU %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); \
	printf("\n"); } } while (0)

/*
 * Reconstrói a linha (1 = rajada/space) amostrando no meio de cada bit, do
 * jeito que um UART faz, e devolve os bytes decodificados. Serve para provar
 * que a sequência de rajadas representa exatamente o que foi pedido.
 */
static int line_at(const nu_burst *b, int n, uint64_t t)
{
	for (int i = 0; i < n; i++)
		if (t >= b[i].start_ns && t < b[i].start_ns + b[i].wire_ns)
			return 1;
	return 0;
}

static size_t decode(const nu_cfg *c, const nu_burst *b, int n, uint64_t t0,
		     size_t nchars, uint8_t *outb)
{
	uint32_t bpc = nu_bits_per_char(c);
	for (size_t k = 0; k < nchars; k++) {
		uint64_t base = (uint64_t)k * bpc;
		uint64_t mid = nu_bit_edge_ns(c, 1) / 2;
		if (!line_at(b, n, t0 + nu_bit_edge_ns(c, base) + mid))
			return k; /* start bit ausente */
		uint8_t v = 0;
		for (uint32_t i = 0; i < c->data_bits; i++)
			if (!line_at(b, n, t0 + nu_bit_edge_ns(c, base + 1 + i) + mid))
				v |= 1u << i;
		if (line_at(b, n, t0 + nu_bit_edge_ns(c, base + 1 + c->data_bits) + mid))
			return k; /* stop bit violado */
		outb[k] = v;
	}
	return nchars;
}

static void test_roundtrip(uint32_t baud, const uint8_t *data, size_t len)
{
	nu_cfg c;
	nu_cfg_default(&c, baud);
	size_t cap = nu_max_bursts_per_char(&c) * len + 1;
	nu_burst *b = calloc(cap, sizeof *b);
	uint8_t *got = calloc(len, 1);
	uint64_t t0 = 1000000, t_end = 0;
	int n = nu_encode(&c, data, len, t0, b, cap, &t_end);
	CHECK(n >= 0, "baud %u: encode falhou: %s", baud, nu_strerror(n));
	if (n >= 0) {
		size_t ok = decode(&c, b, n, t0, len, got);
		CHECK(ok == len && memcmp(got, data, len) == 0,
		      "baud %u: roundtrip falhou no caractere %zu", baud, ok);
		CHECK(t_end == t0 + nu_bit_edge_ns(&c, len * 10),
		      "baud %u: t_end errado", baud);
		for (int i = 0; i < n; i++) {
			CHECK(b[i].frame_len >= NU_MIN_FRAME && b[i].frame_len <= NU_MAX_FRAME,
			      "baud %u: quadro %d fora dos limites (%u)", baud, i, b[i].frame_len);
			if (i)
				CHECK(b[i].start_ns >= b[i-1].start_ns + b[i-1].wire_ns +
				      NU_IFG_BYTES * NU_BYTE_NS_10M,
				      "baud %u: IFG violado entre %d e %d", baud, i - 1, i);
		}
	}
	free(b);
	free(got);
}

static void test_cr_9600(void)
{
	/* 0x0D = 0000 1101 -> LSB primeiro: 1 0 1 1 0 0 0 0
	 * linha: start(S) 1(M) 0(S) 1(M) 1(M) 0 0 0 0(S x4) stop(M)
	 * trechos space: [0,1) [2,3) [5,9) */
	nu_cfg c;
	nu_cfg_default(&c, 9600);
	nu_burst b[16];
	uint8_t cr = 0x0D;
	int n = nu_encode(&c, &cr, 1, 0, b, 16, NULL);
	CHECK(n == 3, "0x0D deveria gerar 3 rajadas, gerou %d", n);
	if (n != 3)
		return;
	CHECK(b[0].start_ns == 0, "rajada 0 início %llu", (unsigned long long)b[0].start_ns);
	CHECK(b[1].start_ns == nu_bit_edge_ns(&c, 2), "rajada 1 início errado");
	CHECK(b[2].start_ns == nu_bit_edge_ns(&c, 5), "rajada 2 início errado");
	/* 1 bit = 104167 ns -> 130 bytes no fio -> quadro de 122 */
	CHECK(b[0].frame_len == 122, "quadro de 1 bit = %u (esperado 122)", b[0].frame_len);
	/* 4 bits = 416667 ns -> 521 bytes -> quadro de 513 */
	CHECK(b[2].frame_len == 513, "quadro de 4 bits = %u (esperado 513)", b[2].frame_len);
	/* erro de quantização < meio byte (400 ns) */
	for (int i = 0; i < 3; i++) {
		uint64_t want = (i == 2 ? 4 : 1) * nu_bit_edge_ns(&c, 1);
		long long diff = (long long)b[i].wire_ns - (long long)want;
		CHECK(diff >= -400 && diff <= 400, "quantização %d = %lld ns", i, diff);
	}
}

static void test_merge_and_patterns(void)
{
	nu_cfg c;
	nu_cfg_default(&c, 9600);
	nu_burst b[16];
	uint8_t v;

	v = 0x00; /* start + 8 zeros = um único trecho de 9 bits */
	CHECK(nu_encode(&c, &v, 1, 0, b, 16, NULL) == 1, "0x00 deveria ser 1 rajada");
	v = 0x55; /* LSB 1 0 1 0 1 0 1 0: S M S M S M S M S M -> 5 rajadas */
	CHECK(nu_encode(&c, &v, 1, 0, b, 16, NULL) == 5, "0x55 deveria ser 5 rajadas");
	v = 0xFF; /* só o start bit */
	CHECK(nu_encode(&c, &v, 1, 0, b, 16, NULL) == 1, "0xFF deveria ser 1 rajada");
	CHECK(nu_encode(&c, NULL, 0, 0, b, 16, NULL) == 0, "string vazia");
}

static void test_limits(void)
{
	nu_cfg c;
	nu_burst b[64];
	uint8_t v = 0xFF;

	/* 19200: 1 bit = 52 us < quadro mínimo 57,6 us (+10,6%) -> aceito com 15% */
	nu_cfg_default(&c, 19200);
	CHECK(nu_encode(&c, &v, 1, 0, b, 64, NULL) == 1, "19200 deveria passar (alongado)");
	CHECK(b[0].frame_len == NU_MIN_FRAME, "19200 deveria usar quadro mínimo");
	c.max_stretch = 0.05;
	CHECK(nu_encode(&c, &v, 1, 0, b, 64, NULL) == NU_ERR_SHORT, "19200 com 5%% deveria falhar");

	nu_cfg_default(&c, 38400);
	CHECK(nu_encode(&c, &v, 1, 0, b, 64, NULL) == NU_ERR_SHORT, "38400 deveria falhar");

	/* 1200: 0x00 = 9 bits de 833 us; 2 bits já não cabem em 1518 B, então
	 * vira 9 quadros, cada um terminando 9,6 us antes da borda seguinte */
	nu_cfg_default(&c, 1200);
	v = 0x00;
	int n = nu_encode(&c, &v, 1, 0, b, 64, NULL);
	CHECK(n == 9, "1200/0x00 deveria virar 9 quadros, virou %d", n);
	if (n == 9)
		CHECK(b[1].start_ns == nu_bit_edge_ns(&c, 1), "divisão fora da borda de bit");

	/* 600: um único bit (1,67 ms) não cabe num quadro */
	nu_cfg_default(&c, 600);
	CHECK(nu_encode(&c, &v, 1, 0, b, 64, NULL) == NU_ERR_LONG, "600 deveria falhar");

	nu_cfg_default(&c, 9600);
	CHECK(nu_encode(&c, &v, 1, 0, NULL, 0, NULL) == NU_ERR_SPACE, "cap 0");
	CHECK(nu_encode(&c, &v, 1, 0, b, 1, NULL) == 1, "cap 1 suficiente para 0x00");
	v = 0x55;
	CHECK(nu_encode(&c, &v, 1, 0, b, 3, NULL) == NU_ERR_SPACE, "cap 3 para 0x55");
	c.baud = 0;
	CHECK(nu_encode(&c, &v, 1, 0, b, 64, NULL) == NU_ERR_ARG, "baud 0");
}

static void test_trim(void)
{
	nu_cfg c;
	nu_cfg_default(&c, 9600);
	c.trim_ns = 8000;
	nu_burst b[4];
	uint8_t v = 0xFF;
	CHECK(nu_encode(&c, &v, 1, 0, b, 4, NULL) == 1, "trim");
	/* 104167 - 8000 = 96167 -> 120 bytes no fio -> 112 */
	CHECK(b[0].frame_len == 112, "trim: quadro %u (esperado 112)", b[0].frame_len);
}

int main(void)
{
	const char *msg = "show version\r\n";
	uint8_t all[256];
	for (int i = 0; i < 256; i++)
		all[i] = (uint8_t)i;

	test_cr_9600();
	test_merge_and_patterns();
	test_limits();
	test_trim();
	uint32_t bauds[] = { 1200, 2400, 4800, 9600, 19200 };
	for (size_t i = 0; i < sizeof bauds / sizeof *bauds; i++) {
		test_roundtrip(bauds[i], (const uint8_t *)msg, strlen(msg));
		test_roundtrip(bauds[i], all, sizeof all);
	}

	if (fails) {
		printf("%d verificação(ões) falharam\n", fails);
		return 1;
	}
	printf("test_encoder: todos os testes passaram\n");
	return 0;
}
