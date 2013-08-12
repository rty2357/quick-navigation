
# workspace directory
WORKSPACE			:=$(dir $(patsubst %/,%,$(PWD)) )

# source directory
# find makefile in subdirectory
PROJECTS			:=$(patsubst ./%/,%,$(dir $(shell find ./ -mindepth 2 -maxdepth 2 -name makefile)))


