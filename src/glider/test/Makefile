# Coludo glider on-board test tasks. Run from this directory (src/glider/test).
#   make test            compile + run all test_*.py on the board, pass/fail report
#   make test-recorder   integration test: UART write -> recorder (needs mpremote + adb)
#   make bench           run the asyncio/PSRAM benchmark on the board
#   make test PORT=...   override the serial port (default /dev/ttyACM0)
PORT ?= /dev/ttyACM0
REC_FILE ?= uart_itest.log

.PHONY: test test-recorder bench clean
test:
	@PORT=$(PORT) bash run_tests.sh

test-recorder:
	@PORT=$(PORT) REC_FILE=$(REC_FILE) bash itest_recorder.sh

bench:
	@mpy-cross -O3 bench_asyncio.py -o /tmp/coludo-bench.mpy \
		&& mpremote connect $(PORT) run bench_asyncio.py

clean:
	@rm -rf /tmp/coludo-tests /tmp/coludo-bench.mpy
