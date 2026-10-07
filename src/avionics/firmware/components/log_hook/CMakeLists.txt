# Coludo project, copyright under MIT license, Alexander Moiseichuk
#
# IDF's own log lines (ESP_LOGx from every C component) into the recorder, next to the console: a vprintf hook that
# formats each line once into a buffer for a Rust callback and hands the untouched arguments on to the console. In C
# because only C can va_copy, and a va_list can be consumed only once.
idf_component_register(SRCS "log_hook.c"
                       INCLUDE_DIRS "include"
                       REQUIRES log panic_capture)
