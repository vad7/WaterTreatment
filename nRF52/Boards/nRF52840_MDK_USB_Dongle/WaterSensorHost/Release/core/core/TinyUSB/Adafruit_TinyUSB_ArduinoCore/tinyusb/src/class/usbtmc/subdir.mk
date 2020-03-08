################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\class\usbtmc\usbtmc_device.c 

C_DEPS += \
.\core\core\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\class\usbtmc\usbtmc_device.c.d 

AR_OBJ += \
.\core\core\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\class\usbtmc\usbtmc_device.c.o 


# Each subdirectory must supply rules for building sources it contributes
core\core\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\class\usbtmc\usbtmc_device.c.o: C:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\class\usbtmc\usbtmc_device.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"C:\eclipse\eclipse\arduinoPlugin\packages\arduino\tools\arm-none-eabi-gcc\7-2017q4/bin/arm-none-eabi-gcc" -mcpu=cortex-m4 -mthumb -c -g -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -Wno-pointer-arith -mfloat-abi=hard -mfpu=fpv4-sp-d16 -u _printf_float -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=64000000 -DARDUINO=10802 -DARDUINO_NRF52840_FEATHER -DARDUINO_ARCH_NRF52 "-DARDUINO_BSP_VERSION=\"0.18.5\""  -DNRF52840_XXAA -DUSBCON -DUSE_TINYUSB -DUSB_VID=0x239A -DUSB_PID=0x8029 "-DUSB_MANUFACTURER=\"Adafruit LLC\"" "-DUSB_PRODUCT=\"Feather nRF52840 Express\"" -DSOFTDEVICE_PRESENT -DARDUINO_NRF52_ADAFRUIT -DNRF52_SERIES -DLFS_NAME_MAX=64 -Ofast -DCFG_DEBUG=0 "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/cmsis/include" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic/nrfx" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic/nrfx/hal" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic/nrfx/mdk" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic/nrfx/soc" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic/nrfx/drivers/include" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic/nrfx/drivers/src" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/nordic/softdevice/s140_nrf52_6.1.1_API/include" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/freertos/Source/include" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/freertos/config" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/freertos/portable/GCC/nrf52" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/freertos/portable/CMSIS/nrf52" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/sysview/SEGGER" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/sysview/Config" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/TinyUSB" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/TinyUSB/Adafruit_TinyUSB_ArduinoCore" "-IC:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/TinyUSB/Adafruit_TinyUSB_ArduinoCore/tinyusb/src"   -I"C:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5\utility" -I"C:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5" -I"C:\Users\Vadim\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.18.5\variants\feather_nrf52840_express" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -D__IN_ECLIPSE__=1 "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '


