# STM32F407G-DISC1 onboard accelerometer test
CC      = arm-none-eabi-gcc
CFLAGS  = -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -ffreestanding -nostdlib \
          -O0 -g -Wall -Wextra
LDFLAGS = -T link.ld -nostdlib

all: fw.elf

fw.elf: startup.c main.c link.ld
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ startup.c main.c
	arm-none-eabi-size $@

flash: fw.elf
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
	  -c "init" -c "reset halt" -c "program fw.elf verify" -c "reset run" -c "shutdown"

clean:
	rm -f fw.elf
