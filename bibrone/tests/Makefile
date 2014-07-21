DEBUG=no

CPP=/home/ohachett/CodeSourcery/bin/arm-none-linux-gnueabi-g++

BIBRONE_PATH=".."

INCLUDES=-I$(BIBRONE_PATH)/include/
DRONEFLAGS= -march=armv7-a
CFLAGS =-W -Wall -Wextra $(DRONEFLAGS) $(INCLUDES)
LDFLAGS =-L$(BIBRONE_PATH)/lib -lbibrone -lm -lpthread

SRC=  $(wildcard *.cpp)
EXEC= $(SRC:.cpp=.elf)
OBJ=  $(SRC:.cpp=.o)

all: $(EXEC)

%.o: %.cpp
ifeq ($(DEBUG),yes)
	$(CPP) -g $(CFLAGS) -o $@ -c $<
else
	$(CPP) $(CFLAGS) -o $@ -c $<
endif

%.elf: %.o
	$(CPP) -static $^ -o $@ $(LDFLAGS)

.PHONY: clean all

clean:
	$(RM) *.elf
	-$(RM) *~
	-$(RM) *.o
	-$(RM) *.bak core Makefile.bak
	-$(RM) $(EXEC)