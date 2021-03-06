
TARGET	:=$(notdir $(patsubst %/,%,$(PWD)) )
SHELL	:=bash
GCC		:=g++
REMOVE	:=rm -rf
MAKEDIR	:=mkdir -p
SRCS	:=$(TARGET).cpp


-include mk/subdir.mk
-include mk/objects.mk
-include mk/launcher.mk

ifeq ($(MAKECMDGOALS),debug)
-include mk/debug.mk
else
ifeq ($(MAKECMDGOALS),debugclean)
-include mk/debug.mk
else
-include mk/options.mk
endif
endif

CFLAGS		:=$(_OPT_OPTION_) $(_WRN_OPTION_) $(_DBG_OPTION_) $(_HDIR_OPTION_)
LDFLAGS		:=$(_LNK_OPTION_) $(_LDIR_OPTION_)


# vpath
vpath
vpath %.cpp $(SRCS_DIR)
vpath %.o 	$(RELEASE_DIR)


.SUFFIXES: .o .cpp
all:rebuild


build:$(RELEASE_DIR) $(OBJS)
	$(GCC) -o"$(RELEASE_DIR)$(TARGET)" $(patsubst %,$(RELEASE_DIR)%,$(OBJS)) $(LDFLAGS)
	$(make-launcher)

rebuild:clean build

debug:rebuild

clean:
	$(REMOVE) $(patsubst %,$(RELEASE_DIR)%,$(OBJS)) $(RELEASE_DIR)$(TARGET)
	$(clean-launcher)

clean-debug:
	$(REMOVE) $(RELEASE_DIR)
	$(clean-launcher)

.cpp.o:
	g++ $(CFLAGS) -c $< -o $(RELEASE_DIR)$@

$(RELEASE_DIR):
	@echo "make directory \"$(RELEASE_DIR)\""
	$(MAKEDIR) $@


.PHONY:all debug clean
