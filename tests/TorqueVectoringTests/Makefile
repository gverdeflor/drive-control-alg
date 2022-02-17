CC = gcc
CFLAGS = -Wall
OBJS = AllTVTests.o CuTest.o TVTests.o torque_vectoring.o

AllTVTests: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o TVTests.exe -lm

AllTVTests.o: AllTVTests.c
	$(CC) $(CFLAGS) -c AllTVTests.c -lm

CuTest.o: ../CuTest.c
	$(CC) $(CFLAGS) -c ../CuTest.c

TVTests.o: TVTests.c
	$(CC) $(CFLAGS) -c TVTests.c -lm

torque_vectoring.o: ../../src/torque_vectoring/torque_vectoring.c ../../src/torque_vectoring/torque_vectoring.h
	$(CC) $(CFLAGS) -c ../../src/torque_vectoring/torque_vectoring.c -lm

clean:
	@echo "Cleaning up..."
	-rm *.exe
	-rm *.o