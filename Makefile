GCC      = /usr/bin/gcc
INCLUDEDIR = src/
CFLAGS  = -Wall -I$(INCLUDEDIR)
SRC = src/attitude_sensor.c src/example.c 
HEADERS = src/attitude_sensor.h  

example: $(SRC) $(HEADERS)
	$(GCC) $(CFLAGS) -o example $(SRC) 
