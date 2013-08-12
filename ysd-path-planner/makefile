
TARGET	:=$(notdir $(patsubst %/,%,$(PWD)) )
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
	@echo "#!$(SHELL_INTRP)" > $(LAUNCHER)
	@echo "$(LAUNCH_CMD)" >> $(LAUNCHER)
	@chmod +x $(LAUNCHER)
	@echo "create launcher"

rebuild:clean build

debug:rebuild

clean:
	$(REMOVE) $(patsubst %,$(RELEASE_DIR)%,$(OBJS)) $(RELEASE_DIR)$(TARGET) $(LAUNCHER)

clean-debug:
	$(REMOVE) $(RELEASE_DIR) $(LAUNCHER)

.cpp.o:
	g++ $(CFLAGS) -c $< -o $(RELEASE_DIR)$@

$(RELEASE_DIR):
	@echo "make directory \"$(RELEASE_DIR)\""
	$(MAKEDIR) $@


.PHONY:all debug clean
