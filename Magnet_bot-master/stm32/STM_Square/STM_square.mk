CC=arm-none-eabi-gcc
AS=arm-none-eabi-as
LD=arm-none-eabi-ld

CCFLAGS=-mcpu=cortex-m0 -mthumb -g
ASFLAGS=-mcpu=cortex-m0 -mthumb -g

OBJS= startup.o main.o 

main.elf: $(OBJS)
	$(LD) $(OBJS) -T linker_script.ld --cref -Map main.map -nostartfiles -o main.elf
	arm-none-eabi-objcopy -O ihex main.elf main.hex
	@echo Success!

main.o: main.c
	$(CC) -c $(CCFLAGS) main.c -o main.o

startup.o: startup.s
	$(AS) $(ASFLAGS) startup.s -asghl=startup.lst -o startup.o

clean: 
	@del $(OBJS) 2>NUL
	@del main.elf main.hex main.map 2>NUL
	@del *.lst 2>NUL
	
Flash_Load:
	STMFlashLoader -c -i STM32F0_5x_3x_64K -e --all -d --fn main.hex --v
 
