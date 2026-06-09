# compiled as luckfox-pico/project/app/recorderd

ifeq ($(APP_PARAM), )
    APP_PARAM:=../Makefile.param
    include $(APP_PARAM)
endif

export LC_ALL=C
SHELL:=/bin/bash

CURRENT_DIR := $(shell pwd)

PKG_NAME := recorderd
PKG_BIN ?= out
PKG_SRC := recorderd.cpp

RECORDERD_DEFS = -Os

RK_APP_CFLAGS = $(RECORDERD_DEFS)
		#	-I $(RK_APP_MEDIA_INCLUDE_PATH)/rkaiq \
		#	-I $(RK_APP_MEDIA_INCLUDE_PATH)/rkaiq/uAPI \
		#	-I $(RK_APP_MEDIA_INCLUDE_PATH)/rkaiq/uAPI2 \
		#	-I $(RK_APP_MEDIA_INCLUDE_PATH)/rkaiq/algos \
		#	-I $(RK_APP_MEDIA_INCLUDE_PATH)/rkaiq/common \

RK_APP_LDFLAGS = # -L $(RK_APP_MEDIA_LIBS_PATH) -L$(RK_APP_OUTPUT)/lib

RK_APP_OPTS += -Wl,-rpath-link,$(RK_APP_PATH_LIB_INCLUDE)/root/usr/lib:$(RK_APP_OUTPUT)/lib

RECORDERD_LD_FLAGS += $(RK_APP_OPTS) \
	$(RK_APP_LDFLAGS) $(RK_APP_CFLAGS) \
	-L$(RK_APP_OUTPUT)/lib -Wl,-Bstatic -Wl,-Bdynamic -Wl,--gc-sections -Wl,--as-needed 

PKG_TARGET := recorderd-build

ifeq ($(PKG_BIN),)
$(error ### $(CURRENT_DIR): PKG_BIN is NULL, Please Check !!!)
endif

all: $(PKG_TARGET)
	@echo "build $(PKG_NAME) done"

recorderd-build: $(PKG_SRC)
	rm -rf $(PKG_BIN); \
	mkdir -p $(PKG_BIN)/bin;
	$(RK_APP_CROSS)-g++ $(PKG_SRC) $(RK_APP_CFLAGS) -s -o $(PKG_BIN)/bin/recorderd $(RECORDERD_LD_FLAGS)
	$(call MAROC_COPY_PKG_TO_APP_OUTPUT, $(RK_APP_OUTPUT), $(PKG_BIN))

clean:
	@rm -rf $(PKG_BIN)

distclean: clean

info:
ifneq ($(RK_RECORDERD),)
	@echo -e "$(C_YELLOW)-------------------------------------------------------------------------$(C_NORMAL)"
	@echo -e "RK_APP_TYPE=$(RK_APP_TYPE)"
	@echo -e "option support as follow:"
	@echo -e "	RK_RECORDERD"
	@echo -e "$(C_YELLOW)-------------------------------------------------------------------------$(C_NORMAL)"
endif
