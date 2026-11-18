/*
 * nicuart - experimento: transmitir RS232 (TX, 8N1) pela porta RJ45 da placa
 * de rede, usando rajadas de quadros 10BASE-T + um detector de envelope
 * passivo. Ver docs/feasibility.md.
 *
 *   nicuart encode [-b baud] [--trim ns] TEXTO      rajadas em CSV (simulação)
 *   nicuart send   -i IF [opções] TEXTO | -f ARQ    envia uma vez
 *   nicuart pty    -i IF [opções] [--link CAMINHO]  porta serial virtual
 *   nicuart jitter -i IF [opções] [-n N] [--hw]     mede a precisão do envio
 *   nicuart rxprobe -i IF --reg R [--mask M]        sonda de RX via MDIO
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <linux/mii.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "encoder.h"
#include "tx.h"

typedef struct {
	nu_cfg cfg;
	nu_tx_opts tx;
	const char *file;
	const char *link;
	int echo;
	unsigned line_delay_ms;
	unsigned count;
	int reg;
	unsigned mask;
	unsigned seconds;
} opts;

static volatile sig_atomic_t g_stop;
static void on_signal(int s) { (void)s; g_stop = 1; }

static void usage(void)
{
	fputs(
"uso: nicuart <comando> [opções] [TEXTO]\n"
"comandos:\n"
"  encode   imprime as rajadas (CSV start_ns,frame_len,wire_ns) para TEXTO\n"
"  send     envia TEXTO (ou -f ARQUIVO) uma vez pela interface\n"
"  pty      cria uma porta serial virtual (/dev/ttyNIC0) ligada à interface\n"
"  jitter   envia um padrão de teste e mede o erro de temporização\n"
"  rxprobe  lê um registrador do PHY (MDIO) o mais rápido possível e registra\n"
"           as mudanças (investigação de RX, fase 5)\n"
"opções:\n"
"  -i, --iface IF        interface (ex.: enp3s0), forçada em 10BASE-T\n"
"  -b, --baud N          1200..19200 (padrão 9600)\n"
"  -m, --mode spin|txtime  temporização (padrão spin)\n"
"      --cpu N           fixa o processo na CPU N (use com isolcpus)\n"
"      --rt              SCHED_FIFO + mlockall\n"
"      --trim NS         encurta cada rajada em NS (compensa o detector)\n"
"      --stretch F       alongamento máximo tolerado de um bit (padrão 0.15)\n"
"      --stop-bits N     1 ou 2\n"
"      --lead US         antecedência da 1a rajada (padrão 500 spin / 5000 txtime)\n"
"  -f, --file ARQ        (send) envia o arquivo linha a linha\n"
"      --line-delay MS   (send) pausa após cada linha (padrão 100)\n"
"      --link CAMINHO    (pty) symlink do escravo (padrão /dev/ttyNIC0)\n"
"      --echo            (pty) eco local, já que não há RX\n"
"  -n, --count N         (jitter) caracteres de teste (padrão 200)\n"
"      --hw              (jitter) timestamps de hardware (i210 etc.)\n"
"      --reg R           (rxprobe) registrador MII (0..31, aceita 0x..)\n"
"      --mask M          (rxprobe) máscara de bits a observar (padrão 0xffff)\n"
"      --seconds S       (rxprobe) duração (padrão 5)\n"
"  -v, --verbose\n"
"TEXTO aceita escapes: \\r \\n \\t \\\\ \\xHH\n", stderr);
}

static size_t unescape(const char *s, uint8_t *out, size_t cap)
{
	size_t n = 0;
	while (*s && n < cap) {
		if (*s != '\\') {
			out[n++] = (uint8_t)*s++;
			continue;
		}
		s++;
		switch (*s) {
		case 'r': out[n++] = '\r'; s++; break;
		case 'n': out[n++] = '\n'; s++; break;
		case 't': out[n++] = '\t'; s++; break;
		case '\\': out[n++] = '\\'; s++; break;
		case 'x': {
			char hex[3] = { 0 };
			if (s[1] && s[2]) {
				hex[0] = s[1];
				hex[1] = s[2];
				out[n++] = (uint8_t)strtoul(hex, NULL, 16);
				s += 3;
			} else {
				s++;
			}
			break;
		}
		case '\0': out[n++] = '\\'; break;
		default: out[n++] = '\\'; break;
		}
	}
	return n;
}

static int parse(int argc, char **argv, opts *o)
{
	enum { O_CPU = 256, O_RT, O_TRIM, O_STRETCH, O_STOP, O_LEAD, O_LINEDELAY,
	       O_LINK, O_ECHO, O_HW, O_REG, O_MASK, O_SECONDS };
	static const struct option lo[] = {
		{ "iface", 1, 0, 'i' }, { "baud", 1, 0, 'b' }, { "mode", 1, 0, 'm' },
		{ "cpu", 1, 0, O_CPU }, { "rt", 0, 0, O_RT }, { "trim", 1, 0, O_TRIM },
		{ "stretch", 1, 0, O_STRETCH }, { "stop-bits", 1, 0, O_STOP },
		{ "lead", 1, 0, O_LEAD }, { "file", 1, 0, 'f' },
		{ "line-delay", 1, 0, O_LINEDELAY }, { "link", 1, 0, O_LINK },
		{ "echo", 0, 0, O_ECHO }, { "count", 1, 0, 'n' }, { "hw", 0, 0, O_HW },
		{ "reg", 1, 0, O_REG }, { "mask", 1, 0, O_MASK },
		{ "seconds", 1, 0, O_SECONDS },
		{ "verbose", 0, 0, 'v' }, { "help", 0, 0, 'h' }, { 0, 0, 0, 0 }
	};
	long lead_us = -1;
	memset(o, 0, sizeof *o);
	nu_cfg_default(&o->cfg, 9600);
	o->tx.cpu = -1;
	o->tx.horizon_ns = 2000000;
	o->link = "/dev/ttyNIC0";
	o->line_delay_ms = 100;
	o->count = 200;
	o->reg = -1;
	o->mask = 0xffff;
	o->seconds = 5;

	int c;
	while ((c = getopt_long(argc, argv, "i:b:m:f:n:vh", lo, NULL)) != -1) {
		switch (c) {
		case 'i': o->tx.ifname = optarg; break;
		case 'b': o->cfg.baud = (uint32_t)strtoul(optarg, NULL, 10); break;
		case 'm':
			if (!strcmp(optarg, "spin")) o->tx.mode = NU_MODE_SPIN;
			else if (!strcmp(optarg, "txtime")) o->tx.mode = NU_MODE_TXTIME;
			else { fprintf(stderr, "modo inválido: %s\n", optarg); return -1; }
			break;
		case O_CPU: o->tx.cpu = atoi(optarg); break;
		case O_RT: o->tx.rt = 1; break;
		case O_TRIM: o->cfg.trim_ns = (uint32_t)strtoul(optarg, NULL, 10); break;
		case O_STRETCH: o->cfg.max_stretch = atof(optarg); break;
		case O_STOP: o->cfg.stop_bits = (uint32_t)atoi(optarg); break;
		case O_LEAD: lead_us = atol(optarg); break;
		case 'f': o->file = optarg; break;
		case O_LINEDELAY: o->line_delay_ms = (unsigned)atoi(optarg); break;
		case O_LINK: o->link = optarg; break;
		case O_ECHO: o->echo = 1; break;
		case 'n': o->count = (unsigned)atoi(optarg); break;
		case O_HW: o->tx.timestamps = 2; break;
		case O_REG: o->reg = (int)strtol(optarg, NULL, 0); break;
		case O_MASK: o->mask = (unsigned)strtoul(optarg, NULL, 0); break;
		case O_SECONDS: o->seconds = (unsigned)atoi(optarg); break;
		case 'v': o->tx.verbose = 1; break;
		default: usage(); return -1;
		}
	}
	if (lead_us < 0)
		lead_us = o->tx.mode == NU_MODE_TXTIME ? 5000 : 500;
	o->tx.lead_ns = (uint64_t)lead_us * 1000;
	return 0;
}

/* ---------------------------------------------------------------- encode */

static int cmd_encode(opts *o, int argc, char **argv)
{
	uint8_t buf[4096];
	size_t len = 0;
	for (int i = optind; i < argc; i++) {
		if (i > optind && len < sizeof buf)
			buf[len++] = ' ';
		len += unescape(argv[i], buf + len, sizeof buf - len);
	}
	size_t cap = nu_max_bursts_per_char(&o->cfg) * (len ? len : 1);
	nu_burst *b = calloc(cap, sizeof *b);
	uint64_t t_end;
	int n = nu_encode(&o->cfg, buf, len, 0, b, cap, &t_end);
	if (n < 0) {
		fprintf(stderr, "encode: %s\n", nu_strerror(n));
		free(b);
		return 1;
	}
	printf("# baud=%u bits=%u%s%u trim_ns=%u end_ns=%llu bytes=%zu\n",
	       o->cfg.baud, o->cfg.data_bits, "N", o->cfg.stop_bits, o->cfg.trim_ns,
	       (unsigned long long)t_end, len);
	printf("start_ns,frame_len,wire_ns\n");
	for (int i = 0; i < n; i++)
		printf("%llu,%u,%u\n", (unsigned long long)b[i].start_ns,
		       b[i].frame_len, b[i].wire_ns);
	free(b);
	return 0;
}

/* ------------------------------------------------------------ transmissão */

typedef struct {
	nu_tx tx;
	nu_burst *b;
	size_t cap;
	uint64_t line_free_at; /* fim do último stop bit agendado */
} sender;

static int sender_open(sender *s, opts *o)
{
	if (!o->tx.ifname) {
		fprintf(stderr, "falta -i INTERFACE\n");
		return -1;
	}
	int r = nu_tx_open(&s->tx, &o->tx);
	if (r < 0) {
		fprintf(stderr, "não consegui abrir %s: %s%s\n", o->tx.ifname, strerror(-r),
			r == -EPERM ? " (precisa de root/CAP_NET_RAW)" : "");
		return -1;
	}
	s->cap = 0;
	s->b = NULL;
	s->line_free_at = 0;
	return 0;
}

/* Agenda e envia data[0..len). Bloqueia até a última rajada sair. */
static int sender_write(sender *s, opts *o, const uint8_t *data, size_t len,
			uint64_t *t0_out, uint32_t *first_id, int *nb)
{
	size_t need = nu_max_bursts_per_char(&o->cfg) * len;
	if (need > s->cap) {
		free(s->b);
		s->b = calloc(need, sizeof *s->b);
		s->cap = need;
	}
	uint64_t t0 = nu_tx_now(&s->tx) + o->tx.lead_ns;
	if (t0 < s->line_free_at)
		t0 = s->line_free_at;
	uint64_t t_end;
	int n = nu_encode(&o->cfg, data, len, t0, s->b, s->cap, &t_end);
	if (n < 0) {
		fprintf(stderr, "encode: %s\n", nu_strerror(n));
		return -1;
	}
	int r = nu_tx_send(&s->tx, s->b, (size_t)n, first_id);
	if (r < 0) {
		fprintf(stderr, "envio: %s\n", strerror(-r));
		return -1;
	}
	s->line_free_at = t_end;
	if (t0_out)
		*t0_out = t0;
	if (nb)
		*nb = n;
	return 0;
}

static void sleep_until(sender *s, uint64_t t)
{
	while (nu_tx_now(&s->tx) < t) {
		struct timespec ts = { 0, 200000 };
		nanosleep(&ts, NULL);
	}
}

static void sender_close(sender *s)
{
	sleep_until(s, s->line_free_at + 1000000);
	nu_tx_drain(&s->tx, NULL, 0);
	if (s->tx.txtime_drops)
		fprintf(stderr, "aviso: etf descartou %llu quadro(s): aumente --lead "
			"ou o delta do etf\n", (unsigned long long)s->tx.txtime_drops);
	nu_tx_close(&s->tx);
	free(s->b);
}

/* ------------------------------------------------------------------ send */

static int cmd_send(opts *o, int argc, char **argv)
{
	sender s;
	if (sender_open(&s, o) < 0)
		return 1;
	int rc = 0;

	if (o->file) {
		FILE *f = fopen(o->file, "rb");
		if (!f) {
			perror(o->file);
			sender_close(&s);
			return 1;
		}
		char line[1024];
		unsigned lines = 0;
		while (fgets(line, sizeof line, f) && !g_stop) {
			size_t l = strcspn(line, "\r\n");
			line[l++] = '\r'; /* consoles esperam CR */
			if (sender_write(&s, o, (uint8_t *)line, l, NULL, NULL, NULL) < 0) {
				rc = 1;
				break;
			}
			/* sem RX não há eco para esperar: só dar tempo ao equipamento */
			sleep_until(&s, s.line_free_at);
			struct timespec ts = { o->line_delay_ms / 1000,
					       (long)(o->line_delay_ms % 1000) * 1000000L };
			nanosleep(&ts, NULL);
			lines++;
		}
		fclose(f);
		fprintf(stderr, "%u linha(s) enviada(s)\n", lines);
	} else {
		uint8_t buf[4096];
		size_t len = 0;
		for (int i = optind; i < argc; i++) {
			if (i > optind && len < sizeof buf)
				buf[len++] = ' ';
			len += unescape(argv[i], buf + len, sizeof buf - len);
		}
		if (!len) {
			fprintf(stderr, "nada para enviar\n");
			rc = 1;
		} else if (sender_write(&s, o, buf, len, NULL, NULL, NULL) < 0) {
			rc = 1;
		} else {
			fprintf(stderr, "%zu byte(s), %llu quadro(s) a %u bps\n", len,
				(unsigned long long)s.tx.sent, o->cfg.baud);
		}
	}
	sender_close(&s);
	return rc;
}

/* ------------------------------------------------------------------- pty */

static uint32_t speed_to_baud(speed_t sp)
{
	switch (sp) {
	case B1200: return 1200;
	case B2400: return 2400;
	case B4800: return 4800;
	case B9600: return 9600;
	case B19200: return 19200;
	case B38400: return 38400;
	default: return 0;
	}
}

static int cmd_pty(opts *o)
{
	sender s;
	if (sender_open(&s, o) < 0)
		return 1;

	int m = posix_openpt(O_RDWR | O_NOCTTY);
	if (m < 0 || grantpt(m) < 0 || unlockpt(m) < 0) {
		perror("posix_openpt");
		return 1;
	}
	const char *slave = ptsname(m);
	/* mantém o escravo aberto: assim o mestre não recebe EIO quando o
	 * terminal (minicom etc.) fecha e reabre a porta */
	int sfd = open(slave, O_RDWR | O_NOCTTY);
	struct termios t;
	tcgetattr(sfd, &t);
	cfmakeraw(&t);
	speed_t sp = B9600;
	switch (o->cfg.baud) {
	case 1200: sp = B1200; break;
	case 2400: sp = B2400; break;
	case 4800: sp = B4800; break;
	case 19200: sp = B19200; break;
	}
	cfsetispeed(&t, sp);
	cfsetospeed(&t, sp);
	tcsetattr(sfd, TCSANOW, &t);

	struct stat st;
	if (lstat(o->link, &st) == 0 && S_ISLNK(st.st_mode))
		unlink(o->link);
	if (symlink(slave, o->link) < 0) {
		fprintf(stderr, "aviso: symlink %s: %s (use %s)\n", o->link,
			strerror(errno), slave);
	}
	fprintf(stderr, "porta pronta: %s -> %s (%u bps, só TX%s)\n", o->link, slave,
		o->cfg.baud, o->echo ? ", eco local" : "");
	fprintf(stderr, "ex.: picocom -b %u %s\n", o->cfg.baud, o->link);

	struct sigaction sa = { .sa_handler = on_signal };
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	uint8_t buf[256];
	while (!g_stop) {
		ssize_t r = read(m, buf, sizeof buf);
		if (r < 0) {
			if (errno == EINTR)
				continue;
			perror("read pty");
			break;
		}
		if (r == 0)
			continue;
		/* respeita o baud que o programa de terminal configurou */
		if (tcgetattr(sfd, &t) == 0) {
			uint32_t want = speed_to_baud(cfgetospeed(&t));
			if (want && want != o->cfg.baud) {
				fprintf(stderr, "baud alterado pelo terminal: %u -> %u\n",
					o->cfg.baud, want);
				o->cfg.baud = want;
			}
		}
		if (sender_write(&s, o, buf, (size_t)r, NULL, NULL, NULL) < 0)
			fprintf(stderr, "descartados %zd byte(s)\n", r);
		if (o->echo && write(m, buf, (size_t)r) < 0)
			perror("eco");
		/* espera o fio ficar livre: é o nosso controle de fluxo */
		sleep_until(&s, s.line_free_at);
	}
	if (lstat(o->link, &st) == 0 && S_ISLNK(st.st_mode))
		unlink(o->link);
	close(sfd);
	close(m);
	sender_close(&s);
	return 0;
}

/* ---------------------------------------------------------------- jitter */

static int cmp_d(const void *a, const void *b)
{
	double x = *(const double *)a, y = *(const double *)b;
	return (x > y) - (x < y);
}

static int cmd_jitter(opts *o)
{
	if (!o->tx.timestamps)
		o->tx.timestamps = 1;
	sender s;
	if (sender_open(&s, o) < 0)
		return 1;

	/* 0x55 = S M S M S M S M S M: 5 rajadas por caractere, o pior caso */
	size_t len = o->count ? o->count : 1;
	uint8_t *data = malloc(len);
	memset(data, 0x55, len);

	uint64_t t0;
	uint32_t first_id;
	int nb;
	if (sender_write(&s, o, data, len, &t0, &first_id, &nb) < 0)
		return 1;
	nu_burst *sched = malloc((size_t)nb * sizeof *sched);
	memcpy(sched, s.b, (size_t)nb * sizeof *sched);
	sleep_until(&s, s.line_free_at + 20000000);

	nu_txts *ts = calloc((size_t)nb + 16, sizeof *ts);
	size_t got = 0, more;
	for (int tries = 0; tries < 50; tries++) {
		more = nu_tx_drain(&s.tx, ts + got, (size_t)nb + 16 - got);
		got += more;
		if (got >= (size_t)nb)
			break;
		struct timespec w = { 0, 10000000 };
		nanosleep(&w, NULL);
	}

	/* mapeia timestamp -> rajada agendada */
	uint64_t *meas = calloc((size_t)nb, sizeof *meas);
	int ids_ok = got && ts[0].id != UINT32_MAX;
	size_t mapped = 0;
	for (size_t i = 0; i < got; i++) {
		long k = ids_ok ? (long)ts[i].id - (long)first_id : (long)i;
		if (k >= 0 && k < nb && !meas[k]) {
			meas[k] = ts[i].ts_ns;
			mapped++;
		}
	}
	if (!ids_ok)
		fprintf(stderr, "aviso: kernel não devolveu OPT_ID; casando por ordem\n");

	/* erro de cada rajada relativo à 1a rajada (start bit) do mesmo
	 * caractere: é isso que o UART do outro lado "sente" */
	uint32_t bpc = nu_bits_per_char(&o->cfg);
	double bit_ns = 1e9 / o->cfg.baud;
	double *err = malloc((size_t)nb * sizeof *err);
	size_t ne = 0;
	long cur_char = -1;
	uint64_t ref_meas = 0, ref_sched = 0;
	for (int k = 0; k < nb; k++) {
		uint64_t bit = ((sched[k].start_ns - t0) * (uint64_t)o->cfg.baud +
				500000000ull) / 1000000000ull;
		long ch = (long)(bit / bpc);
		if (ch != cur_char) {
			cur_char = ch;
			ref_meas = meas[k];
			ref_sched = sched[k].start_ns;
			continue;
		}
		if (!meas[k] || !ref_meas)
			continue;
		err[ne++] = ((double)meas[k] - (double)ref_meas) -
			    ((double)sched[k].start_ns - (double)ref_sched);
	}

	printf("modo=%s timestamps=%s baud=%u caracteres=%zu quadros=%d "
	       "timestamps=%zu casados=%zu\n",
	       o->tx.mode == NU_MODE_TXTIME ? "txtime" : "spin",
	       s.tx.o.timestamps == 2 ? "hw" : "sw", o->cfg.baud, len, nb, got, mapped);
	int rc = 0;
	if (ne == 0) {
		printf("sem medições suficientes (o driver suporta timestamps?)\n");
		rc = 2;
	} else {
		double sum = 0, sq = 0, maxabs = 0;
		for (size_t i = 0; i < ne; i++) {
			sum += err[i];
			sq += err[i] * err[i];
			if (fabs(err[i]) > maxabs)
				maxabs = fabs(err[i]);
		}
		double mean = sum / (double)ne;
		double sd = sqrt(sq / (double)ne - mean * mean);
		double *abs_sorted = malloc(ne * sizeof *abs_sorted);
		for (size_t i = 0; i < ne; i++)
			abs_sorted[i] = fabs(err[i]);
		qsort(abs_sorted, ne, sizeof *abs_sorted, cmp_d);
		double p99 = abs_sorted[(size_t)((double)(ne - 1) * 0.99)];
		double tol = bit_ns / 4.0;
		size_t over = 0;
		for (size_t i = 0; i < ne; i++)
			over += abs_sorted[i] > tol;
		printf("erro relativo ao start bit (ns): média %.0f  desvio %.0f  "
		       "p99 %.0f  máx %.0f\n", mean, sd, p99, maxabs);
		printf("tolerância (1/4 de bit) = %.0f ns; acima dela: %zu de %zu\n",
		       tol, over, ne);
		printf("veredito: %s\n", over == 0 ? "OK para este baud" :
		       "jitter alto demais: tente --rt --cpu, PREEMPT_RT ou -m txtime");
		rc = over == 0 ? 0 : 3;
		free(abs_sorted);
	}
	free(err);
	free(meas);
	free(ts);
	free(sched);
	free(data);
	sender_close(&s);
	return rc;
}

/* --------------------------------------------------------------- rxprobe */

/*
 * Fase 5: o PHY não entrega amostras cruas da linha, mas alguns registradores
 * de status (energy detect, link pulse, polaridade) mudam quando há sinal. Este
 * comando mede quão rápido dá para ler um registrador via MDIO e registra cada
 * mudança dos bits da máscara, para comparar com o RS232 que o equipamento
 * está transmitindo nos pinos de RX.
 */
static int cmd_rxprobe(opts *o)
{
	if (!o->tx.ifname || o->reg < 0 || o->reg > 31) {
		fprintf(stderr, "uso: nicuart rxprobe -i IF --reg R [--mask M] [--seconds S]\n");
		return 1;
	}
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	struct ifreq ifr;
	memset(&ifr, 0, sizeof ifr);
	snprintf(ifr.ifr_name, sizeof ifr.ifr_name, "%s", o->tx.ifname);
	struct mii_ioctl_data *mii = (struct mii_ioctl_data *)&ifr.ifr_data;
	if (ioctl(fd, SIOCGMIIPHY, &ifr) < 0) {
		fprintf(stderr, "SIOCGMIIPHY: %s (o driver expõe MDIO?)\n", strerror(errno));
		close(fd);
		return 1;
	}
	uint16_t phy = mii->phy_id;
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	uint64_t t0 = (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
	uint64_t t_stop = t0 + (uint64_t)o->seconds * 1000000000ull;
	uint64_t reads = 0, changes = 0, now = t0, last_t = t0, max_gap = 0;
	int last = -1;

	printf("# phy=%u reg=0x%02x mask=0x%04x\n# t_us,valor\n", phy, o->reg, o->mask);
	while (now < t_stop) {
		mii->phy_id = phy;
		mii->reg_num = (uint16_t)o->reg;
		if (ioctl(fd, SIOCGMIIREG, &ifr) < 0) {
			fprintf(stderr, "SIOCGMIIREG: %s\n", strerror(errno));
			close(fd);
			return 1;
		}
		clock_gettime(CLOCK_MONOTONIC, &ts);
		now = (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
		if (now - last_t > max_gap)
			max_gap = now - last_t;
		last_t = now;
		reads++;
		int v = mii->val_out & o->mask;
		if (v != last) {
			printf("%.1f,0x%04x\n", (double)(now - t0) / 1e3, v);
			last = v;
			changes++;
		}
	}
	close(fd);
	double secs = (double)(now - t0) / 1e9;
	double period_us = secs * 1e6 / (double)reads;
	fprintf(stderr, "%llu leituras em %.2f s: %.1f us por leitura (pior intervalo "
		"%.1f us), %llu mudanças\n", (unsigned long long)reads, secs, period_us,
		(double)max_gap / 1e3, (unsigned long long)changes);
	fprintf(stderr, "para amostrar RS232 a 9600 bps seria preciso <= 6,5 us por "
		"leitura (16x) ou no mínimo ~35 us (3x)\n");
	return 0;
}

int main(int argc, char **argv)
{
	if (argc < 2 || !strcmp(argv[1], "-h") || !strcmp(argv[1], "--help")) {
		usage();
		return argc < 2 ? 1 : 0;
	}
	const char *cmd = argv[1];
	opts o;
	if (parse(argc - 1, argv + 1, &o) < 0)
		return 1;
	argc -= 1;
	argv += 1;

	if (!strcmp(cmd, "encode"))
		return cmd_encode(&o, argc, argv);
	if (!strcmp(cmd, "send"))
		return cmd_send(&o, argc, argv);
	if (!strcmp(cmd, "pty"))
		return cmd_pty(&o);
	if (!strcmp(cmd, "jitter"))
		return cmd_jitter(&o);
	if (!strcmp(cmd, "rxprobe"))
		return cmd_rxprobe(&o);
	fprintf(stderr, "comando desconhecido: %s\n", cmd);
	usage();
	return 1;
}
