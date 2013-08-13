
#launcher shell script name
LAUNCHER		:=launcher

#lanch option
LAUNCH_CMD		:=./$(RELEASE_DIR)$(TARGET)

#launcher shell script name
LAUNCHER_INC		:=launcher.opt

#lanch option tag
LAUNCH_OPTION_TAG	:=OPTION

#lanch option
LAUNCH_OPTION		:=

#launch command
LAUNCH_SCRIPT = \
echo $(LAUNCH_CMD) \$${$(LAUNCH_OPTION_TAG)} \$$@\n\
$(LAUNCH_CMD) \$${$(LAUNCH_OPTION_TAG)} \$$@

#shell command interpreter
SHELL_INTRP			:=/bin/bash

define make-launcher
	@$(shell) echo -e "#!$(SHELL_INTRP)" > $(LAUNCHER)
	@$(shell) echo -e ". $(LAUNCHER_INC)" >> $(LAUNCHER)
	@$(shell) echo -e "$(LAUNCH_SCRIPT)" >> $(LAUNCHER)
	@chmod +x $(LAUNCHER)
	@$(shell) echo -e "create launcher"
endef

define clean-launcher
	$(REMOVE) $(LAUNCHER)
endef
