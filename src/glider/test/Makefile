# Coludo glider test tasks (MicroPython on the board). Run from this dir (src/glider/test).
#   make test            deploy modules + compile/run all test_*.py on the board, pass/fail
#   make check           verify board setup + report problems (tools/board_check.py) -- needs the
#                        firmware already deployed (run `make test` or ../deploy.sh first)
#   make test-recorder   integration test: UART write -> recorder (needs mpremote + adb)
#   make bench           run the asyncio/PSRAM benchmark on the board
#   make test PORT=...   override the serial port (default /dev/ttyACM0)
PORT ?= /dev/ttyACM0
REC_FILE ?= uart_itest.log

.PHONY: test check test-recorder bench clean
test:
	@PORT=$(PORT) bash run_tests.sh

check:
	@mpremote connect $(PORT) run ../../../tools/board_check.py

test-recorder:
	@PORT=$(PORT) REC_FILE=$(REC_FILE) bash itest_recorder.sh

bench:
	@mpy-cross -O3 bench_asyncio.py -o /tmp/coludo-bench.mpy \
		&& mpremote connect $(PORT) run bench_asyncio.py

clean:
	@rm -rf /tmp/coludo-tests /tmp/coludo-bench.mpy
