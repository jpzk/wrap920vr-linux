GCC      = /usr/bin/gcc
INCLUDEDIR = src/
CFLAGS  = -Wall -I$(INCLUDEDIR) -std=c99 
SRC = src/attitude_sensor.c src/example.c 
HEADERS = src/attitude_sensor.h  

example: $(SRC) $(HEADERS)
	$(GCC) $(CFLAGS) -o example $(SRC) 
