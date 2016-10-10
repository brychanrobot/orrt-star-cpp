CC=g++

CFLAGS=-c -Wall -O3 -march=native -std=c++11
LDFLAGS=-lm -lGL -lGLU -lglfw -lX11 -lXrandr -lXinerama -lXi -lXxf86vm -lXcursor -lpthread -ldl -lboost_system

all: directories orrtstar

directories:
	@mkdir -p bin/geom

orrtstar: bin/main.o bin/OnlineFmtStar.o bin/OnlineRrtStar.o bin/Planner.o bin/Node.o bin/geom/Rect.o
	$(CC) bin/main.o bin/OnlineFmtStar.o bin/OnlineRrtStar.o bin/Planner.o bin/Node.o bin/geom/Rect.o -o bin/orrtstar $(LDFLAGS)

bin/main.o: main.cpp cxxopts.hpp
	$(CC) $(CFLAGS) main.cpp $(LDFLAGS) -o $@

bin/OnlineFmtStar.o: OnlineFmtStar.cpp
	$(CC) $(CFLAGS) OnlineFmtStar.cpp $(LDFLAGS) -o $@

bin/OnlineRrtStar.o: OnlineRrtStar.cpp
	$(CC) $(CFLAGS) OnlineRrtStar.cpp $(LDFLAGS) -o $@

bin/Planner.o: Planner.cpp Halton.hpp
	$(CC) $(CFLAGS) Planner.cpp $(LDFLAGS) -o $@

bin/Node.o: Node.cpp geom/Coord.hpp
	$(CC) $(CFLAGS) Node.cpp -o $@

bin/geom/Rect.o: geom/Rect.cpp geom/Rect.hpp geom/Coord.hpp
	$(CC) $(CFLAGS) geom/Rect.cpp -o $@

clean:
	rm -r bin
