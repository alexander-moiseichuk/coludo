# Coludo local gates. There is no CI by design -- these are the checks, and they run here.
#
#   make check        everything below (run this before you push)
#   make lint         ruff over the whole tree
#   make compile      mpy-cross the board tree (the authority on MicroPython syntax, incl. @viper)
#   make preflight    install + data-consistency gates (also run inside the default scripts)
#   make test-host    the control-hub tests (CPython, no hardware)
#   make docs         regenerate the derived docs (pin map, architecture, telemetry schema)
#   make docs-check   fail if a derived doc is stale
#   make test-board   the on-board suite (needs the board on PORT; slow, not part of `check`)
#
# `check` deliberately excludes test-board: it needs hardware and takes minutes. Run it before a flight.

PORT ?= /dev/ttyACM0
MPYX := tools/mpy-cross.v1.29.0
MARCH := -march=rv32imc

.PHONY: check lint compile preflight test-host docs docs-check test-board clean

check: lint compile preflight docs-check test-host
	@echo ""
	@echo "  ALL LOCAL GATES PASSED  (board suite is separate: make test-board)"

lint:
	@echo "== ruff =="
	@ruff check .

compile:
	@echo "== mpy-cross (board syntax gate) =="
	@if [ ! -x $(MPYX) ]; then echo "  skip: $(MPYX) not present"; else \
		fail=0; \
		for f in src/glider/*.py src/glider/drivers/*.py src/glider/tasks/*.py src/glider/test/*.py; do \
			$(MPYX) -O3 $(MARCH) "$$f" -o /tmp/coludo-gate.mpy 2>/tmp/coludo-gate.err || \
				{ echo "  FAIL $$f"; sed 's/^/    /' /tmp/coludo-gate.err; fail=1; }; \
		done; \
		rm -f /tmp/coludo-gate.mpy /tmp/coludo-gate.err; \
		[ $$fail -eq 0 ] && echo "  every board module compiles" || exit 1; \
	fi

preflight:
	@echo "== preflight (install + data consistency) =="
	@python3 tools/preflight.py

test-host:
	@echo "== control-hub tests (CPython) =="
	@$(MAKE) -s -C src/control/test test

docs:
	@python3 tools/gen_pinmap.py
	@python3 tools/gen_graph.py
	@python3 tools/gen_schema.py

docs-check:
	@echo "== derived docs up to date =="
	@python3 tools/gen_pinmap.py --check
	@python3 tools/gen_graph.py --check
	@python3 tools/gen_schema.py --check

test-board:
	@$(MAKE) -C src/glider/test test PORT=$(PORT)

clean:
	@rm -rf /tmp/coludo-gate.* src/glider/**/__pycache__ tools/__pycache__
