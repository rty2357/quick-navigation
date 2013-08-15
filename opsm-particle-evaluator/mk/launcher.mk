
#launcher shell script name
LAUNCHER		:=launcher

#lanch option
LAUNCH_CMD		:=./$(RELEASE_DIR)$(TARGET)

#launcher shell script name
LAUNCHER_INC		:=launcher.opt

#lanch option tag
LAUNCH_OPTION_TAG	:=OPTION

#lanch config
LAUNCH_CONFIG		:=opsm-particle-evaluator.conf

#lanch option
LAUNCH_OPTION		:=

#launch command
LAUNCH_SCRIPT	:=\
if [ -e $(LAUNCHER_INC) ] ; then\n\
. $(LAUNCHER_INC)\n\
fi\n\n\
if [ -e $(LAUNCH_CONFIG) ] ; then\n\
  $(LAUNCH_CMD) -g $(LAUNCH_CONFIG)  \$${$(LAUNCH_OPTION_TAG)} \$$@\n\
else\n\
  $(LAUNCH_CMD) \$$@ -G $(LAUNCH_CONFIG)\n\
fi

#shell command interpreter
SHELL_INTRP			:=/bin/bash

define make-launcher
	@$(shell) echo -e "#!$(SHELL_INTRP)" > $(LAUNCHER)
	@$(shell) echo -e "$(LAUNCH_SCRIPT)" >> $(LAUNCHER)
	@chmod +x $(LAUNCHER)
	@$(shell) echo -e "create launcher"
endef

define clean-launcher
	$(REMOVE) $(LAUNCHER)
endef


