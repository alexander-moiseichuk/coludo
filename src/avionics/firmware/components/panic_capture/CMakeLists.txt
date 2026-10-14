# Coludo project, copyright under MIT license, Alexander Moiseichuk
#
# The fatal panic's capture for the panic log (crates/panics, doc/specs/avionics-port.md "Panic log"), as Linux's
# ramoops keeps an oops across the reset: the text the panic handler prints, a Rust panic's message before it, and the
# log lines before the panic (a tail the log paths feed, components/log_hook and firmware/src/logging.rs) go into a
# record in RTC (LP) RAM, which survives the panic's reset; the next boot writes it to the `panic` partition with the
# OS up. Nothing is written to flash from the panic handler.
# Two link-time wraps tee the panic handler's output without patching IDF: esp_panic_handler (a panic began) and
# uart_hal_write_txfifo (every character it prints to the UART console passes there).
idf_component_register(SRCS "panic_capture.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_timer hal esp_hw_support freertos)
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_panic_handler" "-Wl,--wrap=uart_hal_write_txfifo")
