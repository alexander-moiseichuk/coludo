# Coludo glider on-board test tasks. Run from this directory (src/glider/test).
#   make test            compile + run all test_*.py on the board, pass/fail report
#   make bench           run the asyncio/PSRAM benchmark on the board
#   make test PORT=...   override the serial port (default /dev/ttyACM0)
PORT ?= /dev/ttyACM0

.PHONY: test bench clean
test:
	@PORT=$(PORT) bash run_tests.sh

bench:
	@mpy-cross -O3 bench_asyncio.py -o /tmp/coludo-bench.mpy \
		&& mpremote connect $(PORT) run bench_asyncio.py

clean:
	@rm -rf /tmp/coludo-tests /tmp/coludo-bench.mpy
