# STM32F407G-DISC1 clock + control-loop benchmark (hard-float / FPU)
CC      = arm-none-eabi-gcc
CFLAGS  = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
          -ffreestanding -nostdlib -O2 -g -Wall -Wextra
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
