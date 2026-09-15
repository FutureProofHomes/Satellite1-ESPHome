# Carries the linker fragment that moves ESPHome's generated main.cpp .bss into PSRAM.
# No sources on purpose: linker fragments are global, so an empty component may map another
# component's archive.
idf_component_register(LDFRAGMENTS "linker.lf")
