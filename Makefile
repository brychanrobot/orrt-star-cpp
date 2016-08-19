CC=g++

CFLAGS=-c -Wall -O3 -std=c++11
LDFLAGS=-lm -lGL -lGLU -lglfw -lX11 -lXrandr -lXinerama -lXi -lXxf86vm -lXcursor -lpthread -ldl -lspatialindex

all: directories orrtstar

directories:
	@mkdir -p bin

orrtstar: bin/main.o bin/OnlineFmtStar.o bin/Node.o
	$(CC) bin/main.o bin/OnlineFmtStar.o bin/Node.o -o bin/orrtstar $(LDFLAGS)

bin/main.o: main.cpp
	$(CC) $(CFLAGS) main.cpp $(LDFLAGS) -o $@

bin/OnlineFmtStar.o: OnlineFmtStar.cpp
	$(CC) $(CFLAGS) OnlineFmtStar.cpp $(LDFLAGS) -o $@

bin/Node.o: Node.cpp
	$(CC) $(CFLAGS) Node.cpp $(LDFLAGS) -o $@

clean:
	rm -r bin
