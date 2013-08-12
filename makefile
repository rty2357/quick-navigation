
-include mk/subdir.mk

FAIL_PROJECT :=

.PHONY: all projects clean

all:projects

projects:
	@for prj in ${PROJECTS}; do			\
		cd $$prj ;				\
		make ;					\
		cd - ;					\
	done						
	@for prj in ${FAIL_PROJECT}; do		\
		echo "Error: fail to build $$prj" ;	\
	done

clean:
	@for prj in ${PROJECTS}; do			\
		cd $$prj ;				\
		make clean;				\
		cd - ;					\
	done
	@for prj in ${FAIL_PROJECT}; do		\
		echo "Error: fail to build $$prj" ;	\
	done



