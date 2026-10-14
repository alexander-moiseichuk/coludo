# EEMBC CoreMark's entry point is a C `main()`, which a Rust binary already defines. Compile Espressif's
# CoreMark component with main renamed to coremark_main, so src/bin/peak.rs can call it.
idf_component_register(REQUIRES espressif__coremark)
idf_component_get_property(coremark_library espressif__coremark COMPONENT_LIB)
target_compile_definitions(${coremark_library} PRIVATE main=coremark_main)
