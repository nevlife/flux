# flux2 wake-protocol model checking. Explorer shared at ../explore.h.
CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra -Werror
MODEL = wake_model
RESTART = -DRESTART_AFTER=1 -DFRAMES=3 -DTICK_BUDGET=6
CRASH = -DCRASH
PARKED = -DSTYLE_PARKED

.PHONY: all check bug vacuous configs clean

all: check bug vacuous configs

# exec and bare are tickless: the wake protocol alone must suffice. restart adds a publisher
# restart and a bounded tick, which drives the re-attach. crash adds a waiter that may die.
# parked is the executor's fallback below Linux 5.16; with restart, the parker is re-aimed.
check: $(MODEL).c ../explore.h
	@set -e; for c in "" "-DSTYLE_BARE" "$(RESTART)" "$(CRASH)" "$(CRASH) -DSTYLE_BARE" \
	  "$(PARKED)" "$(PARKED) $(RESTART)" "$(PARKED) $(CRASH)"; do \
	  $(CC) $(CFLAGS) $$c -o $(MODEL) $<; ./$(MODEL); \
	done

# Each injected defect MUST end in its violation (exit 1).
bug: $(MODEL).c
	@set -e; for b in \
	  "BUG_SAMPLE_AFTER::V_LOST_WAKE" \
	  "BUG_NO_RESYNC:$(RESTART):V_STALE_WAIT" \
	  "BUG_NO_REGATE:$(RESTART):V_GATE" \
	  "BUG_GATE_EARLY:-DSTYLE_BARE:V_SKIP_PARKED" \
	  "BUG_NO_ANNOUNCE:-DSTYLE_BARE:V_SKIP_PARKED" \
	  "BUG_PARK_NOCHECK:-DSTYLE_BARE:V_LOST_WAKE" \
	  "BUG_TWO_STORES:$(CRASH):V_LEAK" \
	  "BUG_GATE_ONE_NUMBER:$(CRASH):V_SKIP_PARKED" \
	  "BUG_REAIM_NO_WAKE:$(PARKED):V_LOST_WAKE" \
	  "BUG_RECLAIM_EARLY:$(PARKED) $(RESTART):V_UNMAPPED" \
	  "BUG_SEEN_AT_POST:$(PARKED):V_LOST_WAKE" \
	  "BUG_HEARD_AT_SLEEP:$(PARKED):V_LOST_WAKE" ; do \
	  def=$${b%%:*}; rest=$${b#*:}; cfg=$${rest%:*}; want=$${rest##*:}; \
	  $(CC) $(CFLAGS) $$cfg -D$$def -o $(MODEL)_bug $<; \
	  echo "expect: $$want with -D$$def"; \
	  if ./$(MODEL)_bug > bug.out 2>&1; then echo "FAIL: -D$$def passed"; cat bug.out; exit 1; fi; \
	  grep -q "VIOLATION $$want" bug.out \
	    || { echo "FAIL: -D$$def did not report $$want"; tail -3 bug.out; exit 1; }; \
	done; rm -f bug.out

# The coverage gate's own teeth: a restart with no frame after it never proves delivery on the
# new segment.
vacuous: $(MODEL).c
	$(CC) $(CFLAGS) -DRESTART_AFTER=3 -DFRAMES=3 -DTICK_BUDGET=6 -o $(MODEL)_vac $<
	@echo "expect: VACUOUS (no frame on the replacement segment)"
	@./$(MODEL)_vac > vac.out 2>&1; test $$? -eq 3 \
	  || { echo "FAIL: expected exit 3 (VACUOUS)"; cat vac.out; exit 1; }
	@rm -f vac.out

configs: $(MODEL).c
	@set -e; for c in \
	  "-DFRAMES=4" \
	  "-DSTYLE_BARE -DFRAMES=4" \
	  "-DRESTART_AFTER=1 -DFRAMES=4 -DTICK_BUDGET=8" \
	  "-DRESTART_AFTER=2 -DFRAMES=4 -DTICK_BUDGET=8" \
	  "$(PARKED) -DFRAMES=4" \
	  "$(PARKED) -DRESTART_AFTER=2 -DFRAMES=4 -DTICK_BUDGET=8" ; do \
	  echo "$$c"; $(CC) $(CFLAGS) $$c -o $(MODEL)_cfg $<; ./$(MODEL)_cfg; \
	done

clean:
	rm -f $(MODEL) $(MODEL)_bug $(MODEL)_vac $(MODEL)_cfg bug.out vac.out
