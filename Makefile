FILES= src/Makefile \
	src/*.h \
	src/*.cpp \
	src/*.cu

handin.tar: $(FILES)
	tar cvf handin.tar $(FILES)

clean:
	(cd code; make clean)
	(cd examples; make clean)
	rm -f *~ handin.tar
