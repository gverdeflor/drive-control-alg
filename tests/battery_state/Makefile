CC = gcc
CFLAGS = -Wall
OBJS = test_battery_state_all.o CuTest.o test_battery_state.o battery_state.o

battery: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o test_battery_state.exe

test_battery_state_all.o: test_battery_state_all.c
	$(CC) $(CFLAGS) -c test_battery_state_all.c

CuTest.o: ../CuTest.c
	$(CC) $(CFLAGS) -c ../CuTest.c

test_battery_state.o: test_battery_state.c
	$(CC) $(CFLAGS) -c test_battery_state.c

battery_state.o: ../../src/battery_state/battery_state.c ../../src/battery_state/battery_state.h
	$(CC) $(CFLAGS) -c ../../src/battery_state/battery_state.c

clean:
	@echo "Cleaning up..."
	-rm *.exe
	-rm *.o