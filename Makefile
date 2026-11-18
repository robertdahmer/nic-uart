CC      ?= gcc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra -Wno-missing-field-initializers
LDLIBS  := -lm
PYTHON  ?= python3

SRC := src/encoder.c src/tx.c src/nicuart.c

all: nicuart

nicuart: $(SRC) src/encoder.h src/tx.h
	$(CC) $(CFLAGS) -o $@ $(SRC) $(LDLIBS)

tests/test_encoder: tests/test_encoder.c src/encoder.c src/encoder.h
	$(CC) $(CFLAGS) -o $@ tests/test_encoder.c src/encoder.c

test: tests/test_encoder nicuart
	./tests/test_encoder

# Integração no loopback do Linux (precisa de root): envia, captura e confere.
test-lo: nicuart
	$(PYTHON) tests/test_loopback.py

# Simula o dongle no ngspice e decodifica a forma de onda com um UART em
# software. Requer ngspice, numpy e matplotlib.
sim: nicuart
	$(PYTHON) sim/run_sim.py --all

clean:
	rm -f nicuart tests/test_encoder
	rm -rf sim/out

.PHONY: all test test-lo sim clean
