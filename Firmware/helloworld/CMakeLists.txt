add_executable(helloworld
    helloworld.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(helloworld pico_stdlib)

pico_enable_stdio_usb(helloworld 1)
pico_enable_stdio_uart(helloworld 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(helloworld)