################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ..\makefile.init

RM := -del 

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include libraries\SPI\subdir.mk
-include core\variant\subdir.mk
-include core\core\utility\subdir.mk
-include core\core\sysview\SEGGER\subdir.mk
-include core\core\sysview\Config\subdir.mk
-include core\core\sysview\subdir.mk
-include core\core\nordic\nrfx\mdk\subdir.mk
-include core\core\nordic\nrfx\hal\subdir.mk
-include core\core\nordic\nrfx\drivers\src\subdir.mk
-include core\core\linker\subdir.mk
-include core\core\freertos\portable\GCC\nrf52\subdir.mk
-include core\core\freertos\portable\CMSIS\nrf52\subdir.mk
-include core\core\freertos\Source\portable\MemMang\subdir.mk
-include core\core\freertos\Source\subdir.mk
-include core\core\avr\subdir.mk
-include core\core\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\portable\nordic\nrf5x\subdir.mk
-include core\core\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\portable\microchip\samd\subdir.mk
-include core\core\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\device\subdir.mk
-include core\core\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\common\subdir.mk
-include core\core\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\class\vendor\subdir.mk
-include core\core\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\class\usbtmc\subdir.mk
-include core\core\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\class\msc\subdir.mk
-include core\core\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\class\midi\subdir.mk
-include core\core\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\class\hid\subdir.mk
-include core\core\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\class\cdc\subdir.mk
-include core\core\TinyUSB\Adafruit_TinyUSB_ArduinoCore\tinyusb\src\subdir.mk
-include core\core\TinyUSB\Adafruit_TinyUSB_ArduinoCore\subdir.mk
-include core\core\TinyUSB\subdir.mk
-include core\core\subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ..\makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
AR += \
G:\Electric\WaterTreatment\nRF52\Boards\nRF52840_MDK_USB_Dongle\WaterSensorHost\Release\arduino.ar \

ELF += \
WaterSensorHost.elf \

HEX += \
WaterSensorHost.hex \


# All Target
all: WaterSensorHost

# Tool invocations
WaterSensorHost: $(HEX)
	@echo 'Building target: $@'
	@echo 'Printing size:'
	"C:\eclipse\arduinoPlugin\packages\arduino\tools\arm-none-eabi-gcc\7-2017q4/bin/arm-none-eabi-size" -A "G:\Electric\WaterTreatment\nRF52\Boards\nRF52840_MDK_USB_Dongle\WaterSensorHost\Release/WaterSensorHost.elf"
	@echo 'Finished building target: $@'
	@echo ' '

G:\Electric\WaterTreatment\nRF52\Boards\nRF52840_MDK_USB_Dongle\WaterSensorHost\Release\arduino.ar: $(AR_OBJ)
	@echo 'Starting archiver'
	"C:\eclipse\arduinoPlugin\packages\arduino\tools\arm-none-eabi-gcc\7-2017q4/bin/arm-none-eabi-ar" rcs     G:\Electric\WaterTreatment\nRF52\Boards\nRF52840_MDK_USB_Dongle\WaterSensorHost\Release/arduino.ar $(AR_OBJ)
	@echo 'Finished building: $@'
	@echo ' '

WaterSensorHost.elf: $(LINK_OBJ) $(AR)
	@echo 'Starting combiner'
	"C:\eclipse\arduinoPlugin\packages\arduino\tools\arm-none-eabi-gcc\7-2017q4/bin/arm-none-eabi-gcc"  "-LG:\Electric\WaterTreatment\nRF52\Boards\nRF52840_MDK_USB_Dongle\WaterSensorHost\Release" -Ofast -Wl,--gc-sections -save-temps  "-LC:\eclipse\arduinoPlugin\packages\adafruit\hardware\nrf52\0.18.5\cores\nRF5/linker" "-Tnrf52840_s140_v6.ld" "-Wl,-Map,G:\Electric\WaterTreatment\nRF52\Boards\nRF52840_MDK_USB_Dongle\WaterSensorHost\Release/WaterSensorHost.map" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -u _printf_float -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align --specs=nano.specs --specs=nosys.specs -o "G:\Electric\WaterTreatment\nRF52\Boards\nRF52840_MDK_USB_Dongle\WaterSensorHost\Release/WaterSensorHost.elf"   $(LINK_OBJ)  -Wl,--start-group -lm  G:\Electric\WaterTreatment\nRF52\Boards\nRF52840_MDK_USB_Dongle\WaterSensorHost\Release/arduino.ar  -Wl,--end-group
	@echo 'Finished building: $@'
	@echo ' '

WaterSensorHost.hex: $(ELF)
	@echo 'Do all objcopy commands'
	${A.RECIPE.OBJCOPY.HEX.PATTERN}
	${A.RECIPE.OBJCOPY.ZIP.PATTERN}
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(PDE_DEPS)$(C_DEPS)$(AR)$(CC_DEPS)$(AR_OBJ)$(C++_DEPS)$(LINK_OBJ)$(CXX_DEPS)$(ASM_DEPS)$(HEX)$(INO_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(ELF)$(CPP_DEPS)
	-@echo ' '

.PHONY: all clean dependents

-include ..\makefile.targets
