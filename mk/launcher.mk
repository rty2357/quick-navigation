
#launcher shell script name
LAUNCHER		:=launcher

#lanch option
LAUNCH_CMD		:=./$(RELEASE_DIR)$(TARGET)

#lanch option
LAUNCH_CONFIG	:=$(PWD)/opsm-position-tracker.conf

#launch command
LAUNCH_SCRIPT	:=if [ -e $(LAUNCH_CONFIG) ] ; then $(LAUNCH_CMD) -g $(LAUNCH_CONFIG) "$$"@; else $(LAUNCH_CMD) "$$"@ -G $(LAUNCH_CONFIG); fi

#shell command interpreter
SHELL_INTRP		:=/bin/bash
