CONTIKI_PROJECT = depcomp
# remove not called functions, -q: keep relocation information
CFLAGS += -ffunction-sections
# LDFLAGS += -Wl,--gc-sections,--print-gc-sections,-q
LDFLAGS += -Wl,--gc-sections,-q
DEFINES=COMPETITION_MODE=0,TINYOS_NODE_ID=1,GLOSSY_PERIOD_MS=50,N_TX=6,N_CH=6,SLEEP_BTW_EVENTS=1,USE_LIGHT_SENSOR=0
ifdef INITIATOR
	# only set INITIATOR_NODE_ID if INITIATOR is defined!
	DEFINES+=INITIATOR_NODE_ID=$(shell echo '$(INITIATOR)' | sed 's/^[0]*//')
else
	# not defined -> define INITIATOR such that flocklab_embedded_image works
	INITIATOR?=001
endif
all: $(CONTIKI_PROJECT)
CONTIKI = ../..
include $(CONTIKI)/Makefile.include
competition:
	@msp430-objcopy -O ihex $(CONTIKI_PROJECT).sky $(CONTIKI_PROJECT).ihex
