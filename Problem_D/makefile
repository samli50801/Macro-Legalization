CC=g++ -g
CFLAGS=-std=c++0x
SOURCES=src/main.cpp src/parser.cpp src/SweepLine.cpp src/Force_Directed.cpp src/FreeSpace.cpp src/TCG.cpp src/tile.cpp src/legalizer.cpp src/cluster.cpp src/CG.cpp src/preprocessor.cpp src/bufferLegalizer.cpp
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE = cadb0133_final

all: $(SOURCES) $(EXECUTABLE)

#liblpsolve55.a -ldl
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) lp_solve/liblpsolve55.a -ldl -o $@

clean:
	rm -rf *.o cadb0133_final
