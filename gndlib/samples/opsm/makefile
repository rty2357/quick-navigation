
#TARGET	:=
GCC		:=g++
REMOVE	:=rm -rf
MAKEDIR	:=mkdir -p
SRCS	:=$(TARGET).cpp
OBJS	:=$(patsubst %.cpp,%.o,$(SRCS))



ifeq (${MAKECMDGOALS}, debug)
#warnning option
_WRN_OPTION_	:=-Wall
#debug option
_DBG_OPTION_	:=-g3
#optimize option
_OPT_OPTION_	:=
#preprocessor option
_PRE_OPTION_	:=
endif
ifeq (${MAKECMDGOALS}, build)
#warnning option
_WRN_OPTION_	:=-Wall
#debug option
_DBG_OPTION_	:=
#optimize option
_OPT_OPTION_	:=-O3
#preprocessor option
_PRE_OPTION_	:=-DNDEBUG=yes
endif

#header directory option
_HDIR_OPTION_	:=-I../../


CFLAGS		=$(_WRN_OPTION_) $(_DBG_OPTION_) $(_OPT_OPTION_) $(_HDIR_OPTION_) $(_PRE_OPTION_)

ifeq (${OS}, Windows_NT)
LDFLAGS		=
else
LDFLAGS		=-lrt
endif

.SUFFIXES: .o .cpp
all:
	make build TARGET=build-map
	make build TARGET=likelihood
	make build TARGET=scan-matching

build:$(OBJS)
	$(GCC) -o $(TARGET) $(OBJS) $(LDFLAGS)

debug-all:
	make debug TARGET=build-map
	make debug TARGET=likelihood
	make debug TARGET=scan-matching

debug:build

clean:
	make cl TARGET=build-map
	make cl TARGET=likelihood
	make cl TARGET=scan-matching

cl:
ifeq (${OS}, Windows_NT)
	rm -rf ${TARGET}.o ${TARGET}.exe
else
	rm -rf ${TARGET}.o ${TARGET}
endif

.cpp.o:
	g++ $(CFLAGS) -c $< -o $@


.PHONY:all build debug debug-all clean cl
