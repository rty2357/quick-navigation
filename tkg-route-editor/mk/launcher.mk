
#launcher shell script name
LAUNCHER		:=launcher

#lanch option
LAUNCH_OPTION	:=-m map

#launch command
LAUNCH_CMD		:=./$(RELEASE_DIR)$(TARGET) $(LAUNCH_OPTION) "$$"@

#shell command interpreter
SHELL_INTRP		:=/bin/bash
