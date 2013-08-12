
#launcher shell script name
LAUNCHER		:=launcher

#lanch option
LAUNCH_OPTION	:=-d ../urg-dev.conf

#launch command
LAUNCH_CMD		:=cd $(RELEASE_DIR); ./$(TARGET) $(LAUNCH_OPTION) "$$"@

#shell command interpreter
SHELL_INTRP		:=/bin/bash
