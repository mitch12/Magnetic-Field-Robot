target remote localhost:3333
set arm abi APCS
monitor reset halt
file main.elf
load
monitor reset

