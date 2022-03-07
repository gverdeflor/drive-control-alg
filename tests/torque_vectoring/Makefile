CC = gcc
CFLAGS = -Wall
OBJS = test_torque_vectoring_all.o CuTest.o test_torque_vectoring.o torque_vectoring.o

torque: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o test_torque_vectoring.exe -lm

test_torque_vectoring_all.o: test_torque_vectoring_all.c
	$(CC) $(CFLAGS) -c test_torque_vectoring_all.c -lm

CuTest.o: ../CuTest.c
	$(CC) $(CFLAGS) -c ../CuTest.c

test_torque_vectoring.o: test_torque_vectoring.c
	$(CC) $(CFLAGS) -c test_torque_vectoring.c -lm

torque_vectoring.o: ../../src/torque_vectoring/torque_vectoring.c ../../src/torque_vectoring/torque_vectoring.h
	$(CC) $(CFLAGS) -c ../../src/torque_vectoring/torque_vectoring.c -lm

clean:
	@echo "Cleaning up..."
	-rm *.exe
	-rm *.o