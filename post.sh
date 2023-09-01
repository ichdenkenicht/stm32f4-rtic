#!/bin/bash
#cargo b --release
cargo objcopy --release -- -O binary out.bin

#sudo dfu-util -a0 -s 0x08000000  -D out.bin
dfu-util -a0 -s 0x08000000  -D out.bin

#st-flash --connect-under-reset --reset write out.bin 0x8000000

