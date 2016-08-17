# Compile option defined:
# LOG_FLT_CTRL_DEBUG_TO_SD : sends extended data packet to nav for SD logging
# MOTOR_TEST : enables motor/propeller response test routine

TARGET := $(notdir $(shell pwd))

MCU   := atmega1284p
F_CPU := 20000000

CCFLAGS   = -std=gnu11 -Wstrict-prototypes
LDFLAGS   = -Ofast -Wall -Wextra -Wundef \
            -fdata-sections -ffunction-sections -fshort-enums \
            -Wl,--relax,--gc-sections,-u,vfprintf -lprintf_flt -lm
LTOFLAGS := -flto -fwhole-program
ALLFLAGS  = -mmcu=$(MCU) -DF_CPU="$(F_CPU)UL"
# PROGRAMMER := avrisp2
PROGRAMMER := atmelice_isp
DUDEFLAGS = -c $(PROGRAMMER) -p $(MCU)

PROGRAM_START := 0x0000
EEPROM_START := 0x0000

# See stdio.h for explanation of various printf implementations. The default
# implemetation used at link time does not include support for floats. Other
# options include:
#   -Wl,-u,vfprintf -lprintf_min
#   -Wl,-u,vfprintf -lprintf_flt -lm

CC   := avr-gcc
CP   := avr-objcopy
DUMP := avr-objdump
DUDE := avrdude

# If the environment variable DEV_BUILD_PATH is set, then the build files will
# be placed there in a named sub-folder, otherwise a build directory will be
# created in the current directory
ifneq ($(DEV_BUILD_PATH),)
  BUILD_PATH := $(DEV_BUILD_PATH)/build/$(TARGET)
else
  BUILD_PATH := build
endif

SOURCES  := $(wildcard *.c)
SOURCES  += $(wildcard *.S)
ASSEMBLY := $(addsuffix .lst, $(addprefix $(BUILD_PATH)/, $(SOURCES)))
HEADERS  := $(wildcard *.h)

ELF := $(BUILD_PATH)/$(TARGET).elf
HEX := $(BUILD_PATH)/$(TARGET).hex
EEP := $(BUILD_PATH)/$(TARGET).eep
LST := $(BUILD_PATH)/$(TARGET).lst

# Rules to make the assembly listings
$(BUILD_PATH)/%.c.lst: %.c
	$(CC) -c $(LDFLAGS) $(CCFLAGS) $(ALLFLAGS) -Wa,-adhlns=$@ -o /dev/null $<

$(BUILD_PATH)/%.S.lst: %.S
	$(CC) -c $(LDFLAGS) $(CCFLAGS) $(ALLFLAGS) -Wa,-adhlns=$@ -o /dev/null $<

# Declare targets that are not files
.PHONY: program write_eeprom clean

all: $(HEX) $(LST)

$(HEX): $(ELF)
	$(CP) -O ihex -R .eeprom --change-section-lma .text=$(PROGRAM_START) $< $@

$(EEP): $(ELF)
	$(CP) -O ihex -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=$(EEPROM_START) $< $@

# Target to make assembly listing of link-time full-program optimized output.
$(LST): $(ELF)
	$(DUMP) -d $(ELF) > $(LST)

# Target to build the .elf file
# NOTE: -lm includes the math library (libm.a)
$(ELF): $(SOURCES) $(HEADERS) $(BUILD_PATH)
	$(CC) $(LTOFLAGS) $(LDFLAGS) $(CCFLAGS) $(ALLFLAGS) -o $@ $(SOURCES) -lm

# Target to program the microprocessor flash only
program: $(HEX)
	$(DUDE) $(DUDEFLAGS) -U flash:w:$(HEX):i

write_eeprom: $(EEP)
	$(DUDE) $(DUDEFLAGS) -U eeprom:w:$(EEP):i

# Target to make assembly listings.
# WARNING!!!: Because this makefile employs link-time optimization, the final
# program may be different from what appears in these files!!!
# Listings are first cleared here since there are no dependency checks.
assembly: clean_assembly $(BUILD_PATH) $(ASSEMBLY)

# Target to clean up the directory (leaving only source)
clean: $(BUILD_PATH)
	rm -f $(HEX) $(ELF) $(EEP) $(LST) $(ASSEMBLY)
	rmdir $(BUILD_PATH)

clean_assembly:
	rm -f $(ASSEMBLY)

$(BUILD_PATH):
	mkdir -p $(BUILD_PATH)