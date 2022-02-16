CC = gcc
CFLAGS = -Wall
OBJS = AllBatteryTests.o CuTest.o BatteryTests.o battery_state.o

AllBatteryTests: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o BatteryTests.exe

AllBatteryTests.o: AllBatteryTests.c
	$(CC) $(CFLAGS) -c AllBatteryTests.c

CuTest.o: ../CuTest.c
	$(CC) $(CFLAGS) -c ../CuTest.c

BatteryTests.o: BatteryTests.c
	$(CC) $(CFLAGS) -c BatteryTests.c

battery_state.o: ../../src/battery_state/battery_state.c ../../src/battery_state/battery_state.h
	$(CC) $(CFLAGS) -c ../../src/battery_state/battery_state.c

clean:
	@echo "Cleaning up..."
	-rm *.exe
	-rm *.o