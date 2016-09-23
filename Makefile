CC=g++

CFLAGS=-c -Wall -O3 -std=c++11
LDFLAGS=-lm -lGL -lGLU -lglfw -lX11 -lXrandr -lXinerama -lXi -lXxf86vm -lXcursor -lpthread -ldl -lboost_system

all: directories orrtstar

directories:
	@mkdir -p bin/geom

orrtstar: bin/main.o bin/OnlineFmtStar.o bin/Node.o bin/geom/Rect.o
	$(CC) bin/main.o bin/OnlineFmtStar.o bin/Node.o bin/geom/Rect.o -o bin/orrtstar $(LDFLAGS)

bin/main.o: main.cpp
	$(CC) $(CFLAGS) main.cpp $(LDFLAGS) -o $@

bin/OnlineFmtStar.o: OnlineFmtStar.cpp
	$(CC) $(CFLAGS) OnlineFmtStar.cpp $(LDFLAGS) -o $@

bin/Node.o: Node.cpp geom/Coord.hpp
	$(CC) $(CFLAGS) Node.cpp -o $@

bin/geom/Rect.o: geom/Rect.cpp geom/Rect.hpp geom/Coord.hpp
	$(CC) $(CFLAGS) geom/Rect.cpp -o $@

clean:
	rm -r bin
