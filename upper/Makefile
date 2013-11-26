###############################################################
# Purposese: Makefile for "joint"
# Author.: duc
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = upper

INCLUDE_DIRS = -I../../../include -I../../../../Framework/include

CXX = g++
#CXXFLAGS += -O2 -DLINUX -Wall $(INCLUDE_DIRS)
CXXFLAGS += -g -DLINUX -Wall $(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall $(INCLUDE_DIRS)
LFLAGS += -lpthread -ljpeg -lrt

OBJECTS = upper.o

all: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

libclean:
	make -C ../../../build clean

distclean: clean libclean

darwin.a:
	make -C ../../../build

$(TARGET): darwin.a $(OBJECTS)
	$(CXX) $(CFLAGS) $(OBJECTS) ../../../lib/darwin.a -o $(TARGET) $(LFLAGS)
	chmod 755 $(TARGET)

