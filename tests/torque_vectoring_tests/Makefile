CC = gcc
# CFLAGS = -c -Wall $(shell pkg-config --cflags check)
# TST_LIBS = $(shell pkg-config --libs check)

tv_tests: tv_tests.o ../../src/torque_vectoring/torque_vectoring.c
	$(CC) -o tv_tests tv_tests.o ../../src/torque_vectoring/torque_vectoring.c `pkg-config --cflags --libs check`



# $(CC) $(CFLAGS) $(TST_LIBS) -o tv_tests tv_tests.o ../../src/torque_vectoring/torque_vectoring.c `pkg-config --cflags --libs check`

# tv_tests: -lm tv_tests.o ../src/torque_vectoring.c
# 	$(CC) -lm -o tv_tests tv_tests.o ../src/torque_vectoring.c `pkg-config --cflags --libs check`

# basetest: basetest.o ../src/torque_vectoring.c
# 	$(CC) -o basetest basetest.o ../src/torque_vectoring.c `pkg-config --cflags --libs check`

clean:
	rm *.o tv_tests