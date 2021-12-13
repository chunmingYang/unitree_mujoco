COMMON = -O2 -I../include -L../bin -mavx -pthread -Wl,-rpath,'$$ORIGIN'
LDFLAGS = -lgsl -lgslcblas -lm 
CFLAGS = -c -Wno-implicit-function-declaration
CC = gcc
ROOT = simulation

all: main_all
	
main_all: main.o
	$(CC) $(COMMON) main.o -w -lmujoco200 -lglew -lGL $(LDFLAGS) ../bin/libglfw.so.3 ../bin/libmujoco200.so ../bin/libglew.so -o ../bin/$(ROOT)
main.o: main.c a1_main.h my_controller.c my_estimator.c
	$(CC) $(COMMON) $(CFLAGS) main.c
clean:
	rm main.o

	
	

