TARGET := $(notdir $(shell pwd))

MCU    := atmega1284p
F_CPU  := 20000000

CCFLAGS   = -std=gnu99 -Wstrict-prototypes
LSTFLAGS  = -Wa,-adhlns=$(LST)
# Temporarily removed -Werror until gcc 4.8.3 comes to Ubuntu
LDFLAGS   = -flto -Ofast -fwhole-program -pedantic -Wall -Wextra -Wundef \
            -fshort-enums -ffreestanding -ffunction-sections -fdata-sections \
            -Wl,--relax,--gc-sections
ALLFLAGS  = -mmcu=$(MCU) -DF_CPU="$(F_CPU)UL"

# LDFLAGS   = -flto -Ofast -pedantic -Werror -Wall -Wextra -Wundef \

CC     := avr-gcc
CP     := avr-objcopy
DUMP   := avr-objdump
DUDE   := avrdude

# If the environment variable DEV_BUILD_PATH is set, then the build files will
# be placed there in a named sub-folder, otherwise a build directory will be
# created in the current directory
ifneq ($(DEV_BUILD_PATH),)
  BUILD_PATH := $(DEV_BUILD_PATH)/build/$(TARGET)
else
  BUILD_PATH := build
endif

SOURCES   = $(wildcard *.c)
SOURCES  += $(wildcard *.S)

LST    := $(BUILD_PATH)/$(TARGET).lst
ELF    := $(BUILD_PATH)/$(TARGET).elf
HEX    := $(BUILD_PATH)/$(TARGET).hex

# Declare targets that are not files
.PHONY: program clean

# Note that without an argument, make simply tries to build the first target
# (not rule), which in this case is this target to build the .hex
$(HEX): $(ELF)
	$(DUMP) -d $(ELF) > $(LST)
	$(CP) -O ihex $(ELF) $(HEX)

# Target to build the .elf file
# NOTE: -lm includes the math library (libm.a)
$(ELF): $(SOURCES)
	mkdir -p $(BUILD_PATH)
	$(CC) $(LDFLAGS) $(CCFLAGS) $(ALLFLAGS) $(LSTFLAGS) -o $(ELF) $(SOURCES) -lm

# Target to program the board
program: $(HEX)
	$(DUDE) -c avrisp2 -p $(MCU) -U flash:w:$(HEX):i

# Target to clean up the directory (leaving only source)
clean:
	rm -f $(HEX) $(ELF) $(LST)
	rmdir $(BUILD_PATH)
