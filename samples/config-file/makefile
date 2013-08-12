
#TARGET	:=
GCC		:=g++
REMOVE	:=rm -rf
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
else
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

LDFLAGS		=

.SUFFIXES: .o .cpp
all:
	make build TARGET=config-file-write
	make build TARGET=config-file-read

build:$(OBJS)
	$(GCC) -o $(TARGET) $(OBJS) $(LDFLAGS)

rebuild:clean build

debug:rebuild

clean:
	${REMOVE} ${TARGET}.o ${TARGET}.exe

.cpp.o:
	g++ $(CFLAGS) -c $< -o $@


.PHONY:all build rebuild clean
