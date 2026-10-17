# flux2 ordering litmus tests under RC11 (GenMC). README.md.
CC ?= cc
CFLAGS ?= -std=c11 -O1 -Wall -Wextra -Werror -pthread
GENMC ?= $(shell command -v genmc || ls $(HOME)/.local/bin/genmc 2>/dev/null)
LITMUS = sb_claim lb_release sb_wake sb_release mp_index mp_epoch seqlock mp_taken

.PHONY: all native check mutate clean

all: native check mutate

# Builds and runs each once on this machine: the files stay valid C where GenMC is missing.
native:
	@set -e; for t in $(LITMUS); do $(CC) $(CFLAGS) -o $$t.bin $$t.c; ./$$t.bin; done; rm -f *.bin

# Every execution under RC11 keeps the assertions and has no data race.
check:
	@if [ -z "$(GENMC)" ]; then echo "SKIP: genmc not found"; exit 0; fi; \
	set -e; for t in $(LITMUS); do echo "$$t"; $(GENMC) -- $$t.c; done

# Teeth: each weakened ordering must break its litmus, except lines marked redundant.
mutate:
	@if [ -z "$(GENMC)" ]; then echo "SKIP: genmc not found"; exit 0; fi; \
	set -e; for t in $(LITMUS); do \
	  python3 ../tools/mutate_orderings.py genmc --strict --genmc "$(GENMC)" $$t.c; done

clean:
	rm -f *.bin
