# build firmware.bin
firmware:
    cargo build --profile embedded --target riscv32imac-unknown-none-elf
    rust-objcopy -O binary target/riscv32imac-unknown-none-elf/embedded/longan-nano-adf4158 firmware.bin

# flash firmware.bin
flash: firmware
    dfu-util -a 0 -s 0x08000000:leave -D firmware.bin
