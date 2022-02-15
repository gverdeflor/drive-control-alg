CC = gcc
# CFLAGS = -c -Wall $(shell pkg-config --cflags check)
TST_LIBS = $(shell pkg-config --cflags --libs check)

TVTests: ../../src/torque_vectoring/torque_vectoring.c
	$(CC) TVTests.c ../CuTest.c AllTorqueVectoringTests.c -o TVTests.exe

clean:
	rm TVTests.exe