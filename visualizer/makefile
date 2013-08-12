
TARGET=visualizer

-include mk/subdir.mk
-include mk/launcher.mk

.PHONY: all build clean

all:build

build:
	@cd $(RELEASE_DIR);	\
	make ;			\
	cd -
	@echo "#!$(SHELL_INTRP)" > $(LAUNCHER)
	@echo "$(LAUNCH_SCRIPT)" >> $(LAUNCHER)
	@chmod +x $(LAUNCHER)
	@echo "create launcher"


clean:
	@cd $(RELEASE_DIR);	\
	make clean ;		\
	cd -
	@if [ -f $(LAUNCHER) ] ; then	\
		rm $(LAUNCHER);	\
	fi

