# flux2 reader-counter model checking. Explorer shared at ../explore.h.
CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra -Werror
MODEL = readers_model
RECLAIM = -DN_PROCS=3 -DN_THREADS=3 -DMAX_CRASHES=1 -DPUB_ABORTS=1
LOAN = -DLOAN_ABORTS=1 -DPUB_FRAMES=4 -DN_THREADS=2

.PHONY: all check bug vacuous configs clean

all: check bug vacuous configs

# Three configs, because one with shared numbers, crashes and loans exhausts the state pool.
check: $(MODEL).c ../explore.h
	$(CC) $(CFLAGS) -o $(MODEL) $<
	./$(MODEL)
	$(CC) $(CFLAGS) $(RECLAIM) -o $(MODEL)_reclaim $<
	./$(MODEL)_reclaim
	$(CC) $(CFLAGS) $(LOAN) -o $(MODEL)_loan $<
	./$(MODEL)_loan

# Each injected defect MUST end in its violation (exit 1).
bug: $(MODEL).c
	@set -e; for b in \
	  "BUG_NO_DEKKER::INV_BYTELOCK" \
	  "BUG_INC_LAST::INV_BYTELOCK" \
	  "BUG_NO_CHECK::INV_BYTELOCK" \
	  "BUG_RECLAIM_NO_LOCK::INV_FLOOR" \
	  "BUG_UNLOCK_EARLY:$(RECLAIM):INV_FLOOR" \
	  "BUG_JOIN_NO_ZERO:$(RECLAIM):INV_LEAK" \
	  "BUG_ABORT_RESTORE:$(LOAN):INV_COHERENT" \
	  "BUG_RAISE_LATE::INV_BYTELOCK" ; do \
	  def=$${b%%:*}; rest=$${b#*:}; cfg=$${rest%:*}; want=$${rest##*:}; \
	  $(CC) $(CFLAGS) $$cfg -D$$def -o $(MODEL)_bug $<; \
	  echo "expect: $$want with -D$$def"; \
	  if ./$(MODEL)_bug > bug.out 2>&1; then echo "FAIL: -D$$def passed"; cat bug.out; exit 1; fi; \
	  grep -q "VIOLATION $$want" bug.out \
	    || { echo "FAIL: -D$$def did not report $$want"; tail -3 bug.out; exit 1; }; \
	done; rm -f bug.out

# The coverage gate's own teeth: one thread holds one slot, so the publisher never starves and
# never reclaims.
vacuous: $(MODEL).c
	$(CC) $(CFLAGS) -DN_PROCS=1 -DN_THREADS=1 -DMAX_CRASHES=1 -o $(MODEL)_vac $<
	@echo "expect: VACUOUS (a lone thread never starves the publisher)"
	@./$(MODEL)_vac > vac.out 2>&1; test $$? -eq 3 \
	  || { echo "FAIL: expected exit 3 (VACUOUS)"; cat vac.out; exit 1; }
	@rm -f vac.out

# Every entry must exit 0, coverage gate included.
configs: $(MODEL).c
	@set -e; for c in \
	  "-DN_SLOTS=3 -DPUB_FRAMES=4 -DN_THREADS=2" \
	  "-DDEPTH=2 -DPUB_FRAMES=3" \
	  "-DSUB_RETRIES=2 -DPUB_ABORTS=3 -DN_THREADS=2" \
	  "$(RECLAIM) -DSUB_RETRIES=2" ; do \
	  echo "$$c"; $(CC) $(CFLAGS) $$c -o $(MODEL)_cfg $<; ./$(MODEL)_cfg; \
	done

clean:
	rm -f $(MODEL) $(MODEL)_reclaim $(MODEL)_loan $(MODEL)_bug $(MODEL)_vac $(MODEL)_cfg bug.out vac.out
