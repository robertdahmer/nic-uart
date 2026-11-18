#include "encoder.h"

void nu_cfg_default(nu_cfg *cfg, uint32_t baud)
{
	cfg->baud = baud;
	cfg->data_bits = 8;
	cfg->stop_bits = 1;
	cfg->trim_ns = 0;
	cfg->max_stretch = 0.15;
	cfg->byte_ns = NU_BYTE_NS_10M;
	cfg->min_frame = NU_MIN_FRAME;
	cfg->max_frame = NU_MAX_FRAME;
}

uint64_t nu_bit_edge_ns(const nu_cfg *cfg, uint64_t idx)
{
	/* arredondado para o ns mais próximo, sem acumular erro */
	return (idx * 1000000000ull + cfg->baud / 2) / cfg->baud;
}

uint32_t nu_bits_per_char(const nu_cfg *cfg)
{
	return 1 + cfg->data_bits + cfg->stop_bits;
}

static int cfg_ok(const nu_cfg *c)
{
	return c && c->baud >= 50 && c->baud <= 1000000 &&
	       c->data_bits >= 5 && c->data_bits <= 8 &&
	       (c->stop_bits == 1 || c->stop_bits == 2) &&
	       c->byte_ns > 0 && c->min_frame >= 1 &&
	       c->max_frame >= c->min_frame && c->max_stretch >= 0.0;
}

/* Bytes no fio (preâmbulo incluído) para cobrir dur_ns, arredondado. */
static uint32_t wire_bytes_for(const nu_cfg *c, uint64_t dur_ns)
{
	return (uint32_t)((dur_ns + c->byte_ns / 2) / c->byte_ns);
}

size_t nu_max_bursts_per_char(const nu_cfg *c)
{
	if (!cfg_ok(c))
		return 0;
	/* pior caso: cada bit space vira um quadro (start + dados) */
	return 1 + c->data_bits;
}

static int push(const nu_cfg *c, uint64_t t_a, uint64_t t_b, int round_down,
		nu_burst *out, size_t cap, size_t *n)
{
	uint64_t dur = t_b - t_a;
	if (c->trim_ns < dur)
		dur -= c->trim_ns;

	/* arredondar para baixo quando outro quadro vem logo depois, para nunca
	 * comer o IFG mínimo */
	uint32_t wire = round_down ? (uint32_t)(dur / c->byte_ns)
				   : wire_bytes_for(c, dur);
	uint32_t min_wire = c->min_frame + NU_PREAMBLE_BYTES;
	uint32_t max_wire = c->max_frame + NU_PREAMBLE_BYTES;

	if (wire < min_wire) {
		/* o quadro mínimo é mais longo que o trecho: alongar se tolerável */
		double stretched = (double)min_wire * c->byte_ns;
		if (stretched > (double)(t_b - t_a) * (1.0 + c->max_stretch))
			return NU_ERR_SHORT;
		wire = min_wire;
	}
	if (wire > max_wire)
		return NU_ERR_LONG;
	if (*n >= cap)
		return NU_ERR_SPACE;
	out[*n].start_ns = t_a;
	out[*n].frame_len = wire - NU_PREAMBLE_BYTES;
	out[*n].wire_ns = wire * c->byte_ns;
	(*n)++;
	return NU_OK;
}

/*
 * Emite rajada(s) para os bits "space" [first, end). Se o trecho não cabe num
 * quadro de 1518 B (baud baixo), ele é dividido sempre numa borda de bit: o
 * silêncio de IFG (9,6 us) fica no fim de um bit, longe do meio onde o UART
 * amostra, e o RC do detector ainda o encobre (ver sim/).
 */
static int emit_run(const nu_cfg *c, uint64_t t0, uint64_t first, uint64_t end,
		    nu_burst *out, size_t cap, size_t *n)
{
	uint64_t max_ns = (uint64_t)(c->max_frame + NU_PREAMBLE_BYTES) * c->byte_ns;
	uint64_t ifg_ns = (uint64_t)NU_IFG_BYTES * c->byte_ns;
	uint64_t a = first;

	while (a < end) {
		uint64_t b = a + 1;
		/* agrupa o máximo de bits inteiros que cabem num quadro */
		while (b < end &&
		       nu_bit_edge_ns(c, b + 1) - nu_bit_edge_ns(c, a) <= max_ns)
			b++;
		uint64_t ta = t0 + nu_bit_edge_ns(c, a);
		uint64_t tb = t0 + nu_bit_edge_ns(c, b);
		if (b < end)
			tb -= ifg_ns; /* abre espaço para o IFG antes do próximo */
		int r = push(c, ta, tb, b < end, out, cap, n);
		if (r < 0)
			return r;
		a = b;
	}
	return NU_OK;
}

int nu_encode(const nu_cfg *c, const uint8_t *data, size_t len, uint64_t t0,
	      nu_burst *out, size_t cap, uint64_t *t_end)
{
	if (!cfg_ok(c) || (len && !data) || (cap && !out))
		return NU_ERR_ARG;

	uint32_t bpc = nu_bits_per_char(c);
	size_t n = 0;
	int in_space = 0;
	uint64_t run_start = 0;

	for (size_t k = 0; k < len; k++) {
		for (uint32_t b = 0; b < bpc; b++) {
			int space;
			if (b == 0)
				space = 1;                         /* start */
			else if (b <= c->data_bits)
				space = !((data[k] >> (b - 1)) & 1); /* LSB primeiro */
			else
				space = 0;                         /* stop */

			uint64_t idx = (uint64_t)k * bpc + b;
			if (space && !in_space) {
				run_start = idx;
				in_space = 1;
			} else if (!space && in_space) {
				int r = emit_run(c, t0, run_start, idx, out, cap, &n);
				if (r < 0)
					return r;
				in_space = 0;
			}
		}
	}
	/* todo caractere termina com stop (mark), então não sobra trecho aberto */
	if (t_end)
		*t_end = t0 + nu_bit_edge_ns(c, (uint64_t)len * bpc);
	return (int)n;
}

const char *nu_strerror(int err)
{
	switch (err) {
	case NU_OK: return "ok";
	case NU_ERR_ARG: return "configuração inválida";
	case NU_ERR_SPACE: return "buffer de rajadas insuficiente";
	case NU_ERR_SHORT: return "baud alto demais: bit menor que o quadro Ethernet mínimo";
	case NU_ERR_LONG: return "baud baixo demais: um bit não cabe num quadro de 1518 bytes";
	default: return "erro desconhecido";
	}
}
