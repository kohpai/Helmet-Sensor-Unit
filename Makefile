PROJECT_NAME := smart_helmet

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) )

TEMPLATE_PATH = ../../../components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO :=
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(abspath ../../../components/libraries/util/app_error.c) \
$(abspath ../../../components/libraries/util/app_error_weak.c) \
$(abspath ../../../components/libraries/fifo/app_fifo.c) \
$(abspath ../../../components/libraries/twi/app_twi.c) \
$(abspath ../../../components/libraries/timer/app_timer.c) \
$(abspath ../../../components/libraries/trace/app_trace.c) \
$(abspath ../../../components/libraries/util/app_util_platform.c) \
$(abspath ../../../components/libraries/uart/retarget.c) \
$(abspath ../../../components/libraries/uart/app_uart_fifo.c) \
$(abspath ../../../components/drivers_nrf/hal/nrf_adc.c) \
$(abspath ../../../components/libraries/util/nrf_assert.c) \
$(abspath ../../../components/libraries/fstorage/fstorage.c) \
$(abspath ../../../components/libraries/util/nrf_log.c) \
$(abspath ../../../components/drivers_nrf/delay/nrf_delay.c) \
$(abspath ../../../components/drivers_nrf/adc/nrf_drv_adc.c) \
$(abspath ../../../components/libraries/sensorsim/sensorsim.c) \
$(abspath ../../../components/drivers_nrf/common/nrf_drv_common.c) \
$(abspath ../../../components/drivers_nrf/twi_master/nrf_drv_twi.c) \
$(abspath ../../../components/drivers_nrf/pstorage/pstorage.c) \
$(abspath ../../../components/drivers_nrf/uart/nrf_drv_uart.c) \
$(abspath ../../../components/drivers_nrf/nrf_soc_nosd/nrf_nvic.c) \
$(abspath ../../../components/drivers_nrf/nrf_soc_nosd/nrf_soc.c) \
$(abspath ../../../components/drivers_nrf/clock/nrf_drv_clock.c) \
$(abspath ../../../components/drivers_nrf/gpiote/nrf_drv_gpiote.c) \
$(abspath ../../bsp/bsp.c) \
$(abspath src/main.c) \
$(abspath src/mpu9250.c) \
$(abspath src/sensors.c) \
$(abspath src/bluetooth.c) \
$(abspath src/helmet_sensor_unit_service.c) \
$(abspath src/error_event.c) \
$(abspath src/circular_queue.c) \
$(abspath src/time.c) \
$(abspath ../../../components/ble/common/ble_advdata.c) \
$(abspath ../../../components/ble/ble_advertising/ble_advertising.c) \
$(abspath ../../../components/ble/common/ble_conn_params.c) \
$(abspath ../../../components/ble/common/ble_srv_common.c) \
$(abspath ../../../components/ble/device_manager/device_manager_peripheral.c) \
$(abspath ../../../components/toolchain/system_nrf51.c) \
$(abspath ../../../components/softdevice/common/softdevice_handler/softdevice_handler.c) \

#assembly files common to all targets
ASM_SOURCE_FILES  = $(abspath ../../../components/toolchain/gcc/gcc_startup_nrf51.s)

#includes common to all targets
INC_PATHS += -I$(abspath inc)
INC_PATHS += -I$(abspath ../../../components/drivers_nrf/config)
INC_PATHS += -I$(abspath ../../../components/softdevice/s130/headers)
INC_PATHS += -I$(abspath ../../../components/drivers_nrf/nrf_soc_nosd)
INC_PATHS += -I$(abspath ../../../components/libraries/twi)
INC_PATHS += -I$(abspath ../../../components/libraries/timer)
INC_PATHS += -I$(abspath ../../../components/libraries/fstorage/config)
INC_PATHS += -I$(abspath ../../../components/drivers_nrf/delay)
INC_PATHS += -I$(abspath ../../../components/libraries/util)
INC_PATHS += -I$(abspath ../../../components/ble/device_manager)
INC_PATHS += -I$(abspath ../../../components/ble/device_manager/config)
INC_PATHS += -I$(abspath ../../../components/drivers_nrf/uart)
INC_PATHS += -I$(abspath ../../../components/ble/common)
INC_PATHS += -I$(abspath ../../../components/libraries/sensorsim)
INC_PATHS += -I$(abspath ../../../components/drivers_nrf/pstorage)
INC_PATHS += -I$(abspath ../../../components/drivers_nrf/pstorage/config)
INC_PATHS += -I$(abspath ../../../components/libraries/uart)
INC_PATHS += -I$(abspath ../../../components/device)
INC_PATHS += -I$(abspath ../../../components/libraries/button)
INC_PATHS += -I$(abspath ../../../components/libraries/fstorage)
INC_PATHS += -I$(abspath ../../../components/libraries/experimental_section_vars)
INC_PATHS += -I$(abspath ../../../components/drivers_nrf/gpiote)
INC_PATHS += -I$(abspath ../../../components/drivers_nrf/twi_master)
INC_PATHS += -I$(abspath ../../../components/drivers_nrf/adc)
INC_PATHS += -I$(abspath ../../../components/drivers_nrf/clock)
INC_PATHS += -I$(abspath ../../../components/libraries/fifo)
INC_PATHS += -I$(abspath ../../bsp)
INC_PATHS += -I$(abspath ../../../components/toolchain/CMSIS/Include)
INC_PATHS += -I$(abspath ../../../components/drivers_nrf/hal)
INC_PATHS += -I$(abspath ../../../components/toolchain/gcc)
INC_PATHS += -I$(abspath ../../../components/toolchain)
INC_PATHS += -I$(abspath ../../../components/drivers_nrf/common)
INC_PATHS += -I$(abspath ../../../components/ble/ble_advertising)
INC_PATHS += -I$(abspath ../../../components/softdevice/s130/headers/nrf51)
INC_PATHS += -I$(abspath ../../../components/libraries/trace)
INC_PATHS += -I$(abspath ../../../components/softdevice/common/softdevice_handler)

OBJECT_DIRECTORY = obj
LISTING_DIRECTORY = lst
OUTPUT_BINARY_DIRECTORY = bin

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
#CFLAGS  = -DNRF_LOG_USES_UART=1
CFLAGS += -DBOARD_BLE400
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS130
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSWI_DISABLE0
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -O3 -g3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums
# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys
LDFLAGS += -u _printf_float

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
#ASMFLAGS += -DNRF_LOG_USES_UART=1
ASMFLAGS += -DBOARD_BLE400
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS130
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSWI_DISABLE0

#default target - first one defined
#default: clean nrf51822_xxac_s130
default: nrf51822_xxac_s130

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51822_xxac_s130

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51822_xxac_s130
	@echo 	flash_softdevice

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf51822_xxac_s130: OUTPUT_FILENAME := nrf51822_xxac_s130
nrf51822_xxac_s130: LINKER_SCRIPT=nrf5-ble-tutorial-advertising.ld

nrf51822_xxac_s130: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).elf
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).elf
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Assembly file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<
# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).elf: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).elf
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).elf
## Create binary .bin file from the .elf file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).elf
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).elf $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .elf file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).elf
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).elf $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).elf $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .elf file
genhex:
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).elf $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).elf
	-@echo ''

clean:
	$(RM) $(OBJECT_DIRECTORY)/* $(OUTPUT_BINARY_DIRECTORY)/* $(LISTING_DIRECTORY)/*

cleanobj:
	$(RM) $(OBJECT_DIRECTORY)/r

reset: nrf51822_xxac_s130
	openocd -s /usr/share/openocd/scripts -f interface/stlink-v2-1.cfg -f target/nrf51.cfg -c "init" -c "reset run" -c shutdown

flash: nrf51822_xxac_s130
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.bin
	openocd -s /usr/share/openocd/scripts -f interface/stlink-v2-1.cfg -f target/nrf51.cfg -c "init" -c "reset halt" -c "sleep 100" -c "wait_halt 2" -c "flash write_image $(OUTPUT_BINARY_DIRECTORY)/$<.bin 0x1B000" -c "sleep 100" -c "reset run" -c shutdown

## Flash softdevice
flash_softdevice:
	@echo Flashing: s130_nrf51_2.0.0_softdevice.hex
	openocd -s /usr/share/openocd/scripts -f interface/stlink-v2-1.cfg -f target/nrf51.cfg -c "init" -c "reset halt" -c "sleep 100" -c "wait_halt 2" -c "flash write_image erase ../../../components/softdevice/s130/hex/s130_nrf51_2.0.0_softdevice.hex 0" -c "sleep 100" -c "reset run" -c shutdown
