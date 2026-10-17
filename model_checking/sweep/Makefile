# flux2 /dev/shm naming and sweep model checking. Explorer shared at ../explore.h.
CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra -Werror
MODEL = sweep_model
RESTART = -DN_PUBS=2 -DN_SUBS=1 -DN_SWEEPERS=1
USERS = -DN_SUBS=2 -DN_SWEEPERS=1
CLEAN = -DMAX_CRASHES=0

.PHONY: all check bug vacuous configs clean

all: check bug vacuous configs

# Two sweepers against a publisher and a subscriber; two publishers on one signpost (a restart,
# or one refused) with a subscriber and a sweeper; three users of one signpost and a sweeper. Each
# with one death anywhere.
check: $(MODEL).c ../explore.h
	@set -e; for c in "" "$(RESTART)" "$(USERS)"; do \
	  $(CC) $(CFLAGS) $$c -o $(MODEL) $<; ./$(MODEL); \
	done

# Each injected defect MUST end in its violation (exit 1).
bug: $(MODEL).c
	@set -e; for b in \
	  "BUG_RM_NO_CHECK::INV_UNLINK_LIVE" \
	  "BUG_LOCK_AFTER_CHECK::INV_UNLINK_LIVE" \
	  "BUG_RM_NO_LOCK::INV_UNLINK_LIVE" \
	  "BUG_DROP_NO_LOCK::INV_UNLINK_LIVE" \
	  "BUG_OPEN_NO_CHECK::INV_REBOUND" \
	  "BUG_CREATE_NO_CHECK::INV_REBOUND" \
	  "BUG_DROP_NO_RETRY::INV_LEAK" \
	  "BUG_DROP_NO_RECHECK:$(USERS):INV_UNLINK_LIVE" ; do \
	  def=$${b%%:*}; rest=$${b#*:}; cfg=$${rest%:*}; want=$${rest##*:}; \
	  $(CC) $(CFLAGS) $$cfg -D$$def -o $(MODEL)_bug $<; \
	  echo "expect: $$want with -D$$def"; \
	  if ./$(MODEL)_bug > bug.out 2>&1; then echo "FAIL: -D$$def passed"; cat bug.out; exit 1; fi; \
	  grep -q "VIOLATION $$want" bug.out \
	    || { echo "FAIL: -D$$def did not report $$want"; tail -3 bug.out; exit 1; }; \
	done; rm -f bug.out

# The coverage gate's own teeth: a lone publisher with nothing left over removes nothing.
vacuous: $(MODEL).c
	$(CC) $(CFLAGS) -DN_SWEEPERS=0 -DN_SUBS=0 -DLEFTOVER=0 -DMAX_CRASHES=0 -o $(MODEL)_vac $<
	@echo "expect: VACUOUS (nothing to remove)"
	@./$(MODEL)_vac > vac.out 2>&1; test $$? -eq 3 \
	  || { echo "FAIL: expected exit 3 (VACUOUS)"; cat vac.out; exit 1; }
	@rm -f vac.out

# Every entry must exit 0, coverage gate included.
configs: $(MODEL).c
	@set -e; for c in \
	  "$(CLEAN)" \
	  "-DN_EPOCHS=3 -DN_SWEEPERS=1" \
	  "-DLEFTOVER=0" ; do \
	  echo "$$c"; $(CC) $(CFLAGS) $$c -o $(MODEL)_cfg $<; ./$(MODEL)_cfg; \
	done

clean:
	rm -f $(MODEL) $(MODEL)_bug $(MODEL)_vac $(MODEL)_cfg bug.out vac.out
