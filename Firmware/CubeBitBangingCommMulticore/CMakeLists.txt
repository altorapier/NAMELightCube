add_executable(CubeBitBangingCommMulticore
CubeBitBangingCommMulticore.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(CubeBitBangingCommMulticore
pico_stdlib
pico_multicore)

pico_enable_stdio_usb(CubeBitBangingCommMulticore 1)
pico_enable_stdio_uart(CubeBitBangingCommMulticore 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(CubeBitBangingCommMulticore)