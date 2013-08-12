
# workspace directory
WORKSPACE			:=$(dir $(patsubst %/,%,$(PWD)) )

# source directory
SRCS_DIR			:=src/

# search header directory (relative directory path from workspace)
HEADER_DIR_LIST		:=gndlib/ ssmtype/

# search header directory (relative directory path from workspace)
LIB_DIR_LIST		:=

# target release directory
ifeq ($(MAKECMDGOALS),debug)
RELEASE_DIR			:=Debug/
else
ifeq ($(MAKECMDGOALS),clean-debug)
RELEASE_DIR			:=Debug/
else
RELEASE_DIR			:=Release/
endif
endif

