################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5\freertos\Source\portable\MemMang\heap_3.c 

C_DEPS += \
.\core\core\freertos\Source\portable\MemMang\heap_3.c.d 

AR_OBJ += \
.\core\core\freertos\Source\portable\MemMang\heap_3.c.o 


# Each subdirectory must supply rules for building sources it contributes
core\core\freertos\Source\portable\MemMang\heap_3.c.o: C:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5\freertos\Source\portable\MemMang\heap_3.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"C:\eclipse\arduinoPlugin\packages\arduino\tools\arm-none-eabi-gcc\7-2017q4/bin/arm-none-eabi-gcc" -mcpu=cortex-m4 -mthumb -c -g -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -Wno-pointer-arith -mfloat-abi=hard -mfpu=fpv4-sp-d16 -u _printf_float -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=64000000 -DARDUINO=10812 -DARDUINO_NRF52840_FEATHER -DARDUINO_ARCH_NRF52 "-DARDUINO_BSP_VERSION=\"0.18.5\""  -DNRF52840_XXAA -DUSBCON -DUSE_TINYUSB -DUSB_VID=0x239A -DUSB_PID=0x8029 '-DUSB_MANUFACTURER=\"Adafruit LLC\"' '-DUSB_PRODUCT=\"Feather nRF52840 Express\"' -DSOFTDEVICE_PRESENT -DARDUINO_NRF52_ADAFRUIT -DNRF52_SERIES -DLFS_NAME_MAX=64 -Ofast -DCFG_DEBUG=0 "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/cmsis/include" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic/nrfx" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic/nrfx/hal" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic/nrfx/mdk" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic/nrfx/soc" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic/nrfx/drivers/include" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic/nrfx/drivers/src" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic/softdevice/s140_nrf52_6.1.1_API/include" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/freertos/Source/include" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/freertos/config" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/freertos/portable/GCC/nrf52" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/freertos/portable/CMSIS/nrf52" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/sysview/SEGGER" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/sysview/Config" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/TinyUSB" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/TinyUSB/Adafruit_TinyUSB_ArduinoCore" "-IC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/TinyUSB/Adafruit_TinyUSB_ArduinoCore/tinyusb/src"   -I"C:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5\utility" -I"C:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5" -I"C:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\variants\feather_nrf52840_express" -I"C:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\libraries\SPI" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -D__IN_ECLIPSE__=1 "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

