add_executable(CubeBitBanging
CubeBitBanging.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(CubeBitBanging pico_stdlib)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(CubeBitBanging)