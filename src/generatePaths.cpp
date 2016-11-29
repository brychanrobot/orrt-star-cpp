#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <set>
#include "color.hpp"
#include "cxxopts.hpp"
#include "planning/AStar.hpp"
#include "planning/OnlineFmtStar.hpp"
#include "planning/OnlineRrtStar.hpp"
#include "planning/utils.hpp"

using namespace std;

const double OBSTACLE_PADDING = 5;

static void onError(int error, const char* description) { fputs(description, stderr); }

static void onKey(GLFWwindow* window, int key, int scancode, int action, int mods) {
	switch (key) {
		case GLFW_KEY_ESCAPE:
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
	}
}

void initDisplay(int width, int height, float ratio) {
	glViewport(0, 0, width, height);
	glClear(GL_COLOR_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, width, 0, height, -1.f, 1.f);
	glScalef(1, -1, 1);
	glTranslatef(0, -height, 0);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void drawPoint(Coord point, double radius) {
	glEnable(GL_POINT_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glPointSize(radius);
	glColor3d(1.0, 1.0, 0);

	glBegin(GL_POINTS);
	glVertex2d(point.x(), point.y());
	glEnd();
}

void drawPath(deque<Coord>& path) {
	glEnable(GL_LINE_SMOOTH);
	glLineWidth(3);
	glColor3d(0.0, 1.0, 0.2);

	glBegin(GL_LINE_STRIP);
	for (auto point : path) {
		glVertex2d(point.x(), point.y());
	}
	glEnd();

	glDisable(GL_LINE_SMOOTH);
}

void drawObstacles(vector<shared_ptr<Rect>>& obstacleRects) {
	glColor3d(0.0, 1.0, 1.0);
	glBegin(GL_QUADS);
	for (const auto& obstacle : obstacleRects) {
		glVertex2d(obstacle->topLeft.x() + OBSTACLE_PADDING, obstacle->topLeft.y() + OBSTACLE_PADDING);
		glVertex2d(obstacle->bottomRight.x() - OBSTACLE_PADDING, obstacle->topLeft.y() + OBSTACLE_PADDING);
		glVertex2d(obstacle->bottomRight.x() - OBSTACLE_PADDING, obstacle->bottomRight.y() - OBSTACLE_PADDING);
		glVertex2d(obstacle->topLeft.x() + OBSTACLE_PADDING, obstacle->bottomRight.y() - OBSTACLE_PADDING);
	}
	glEnd();
}

void display(shared_ptr<Node>& root, shared_ptr<Node>& endNode, deque<Coord>& bestPath, vector<shared_ptr<Rect>>& obstacleRects) {
	drawObstacles(obstacleRects);
	drawPath(bestPath);

	drawPoint(root->coord, 20);
	drawPoint(endNode->coord, 20);
}

void saveMap(string filename, vector<vector<bool>>& map) {
	ofstream f(filename, std::ofstream::out);
	f << "[";
	for (int r = 0; r < map.size(); r++) {
		f << "[";
		for (int c = 0; c < map[r].size(); c++) {
			f << map[r][c] ? 1 : 0;
			if (c < map[r].size() - 1) {
				f << ", ";
			}
		}
		if (r < map.size() - 1) {
			f << "],";
		} else {
			f << "]";
		}
	}
	f << "]";
	f.close();
}

void appendPath(string filename, Coord& start, Coord& end, deque<Coord>& path) {
	ofstream f(filename, std::ofstream::out | std::ofstream::app);
	if (f.tellp() == 0) {
		f << "[";
	} else {
		f << ", ";
	}

	f << "[[" << start.x() << "," << start.y() << "], [" << end.x() << "," << end.y() << "], [";
	for (int c = 0; c < path.size(); c++) {
		if (c == 0) {
			f << "[";
		} else {
			f << ",[";
		}
		f << path[c].x() << "," << path[c].y() << "]";
	}
	// f.seekp(2, ios_base::end);
	// f << "\b";
	// f.seekp(f.tellp() - 1);
	// f.write("],", 2);
	f << "]]";
	f.close();
}

int main(int argc, char* argv[]) {
	srand(time(NULL));

	int width = 700;
	int height = 700;
	bool isFullscreen = false;
	int monitorNum = 0;
	bool usePseudoRandom = false;
	double replanFrequency = -1;

	// clang-format off
	cxxopts::Options options("OnlineRRT*", "A cool program for cool things");
	options.add_options()
		("f,fullscreen", "Enable Fullscreen", cxxopts::value(isFullscreen))
		("m,monitor", "Set Monitor Number", cxxopts::value(monitorNum))
		("p,pr", "Use pseudo-random numbers", cxxopts::value(usePseudoRandom))
		("r,replan", "Replan frequency", cxxopts::value(replanFrequency));
	// clang-format on

	options.parse(argc, argv);

	GLFWwindow* window;
	glfwSetErrorCallback(onError);
	if (!glfwInit()) exit(EXIT_FAILURE);

	GLFWmonitor* monitor = NULL;
	// if (options["fullscreen"].as<bool>()) {

	if (isFullscreen) {
		int count;
		GLFWmonitor** monitors = glfwGetMonitors(&count);
		monitor = monitors[monitorNum];
		auto vidMode = glfwGetVideoMode(monitor);
		width = vidMode->width;
		height = vidMode->height;
	}

	window = glfwCreateWindow(width, height, "Simple example", monitor, NULL);
	if (!window) {
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwMakeContextCurrent(window);
	glfwSetKeyCallback(window, onKey);

	float ratio = width / (float)height;
	initDisplay(width, height, ratio);

	/*
	vector<shared_ptr<Rect>> obstacleRects;
	generateObstacleRects(width, height, 10, obstacleRects, OBSTACLE_PADDING);

	vector<vector<bool>> obstacleHash(height, vector<bool>(width, false));
	generateObstacleHash(obstacleRects, obstacleHash);

	AStar* planner = new AStar(&obstacleHash, &obstacleRects, width, height, usePseudoRandom);
	*/
	AStar* planner = NULL;
	vector<shared_ptr<Rect>> obstacleRects;
	vector<vector<bool>> obstacleHash;

	auto lastFrame = glfwGetTime();
	auto frameInterval = 1.0 / 30.0;
	int replanCount = 0;

	auto lastReplan = glfwGetTime();
	auto replanInterval = 1.0 / replanFrequency;
	printf("%.2f\n", replanInterval);

	int mapNum = 0;

	while (!glfwWindowShouldClose(window)) {
		if (replanCount % 10 == 0) {
			/*for (auto rect : obstacleRects) {
			    delete rect;
			}*/
			if (planner != NULL) {
				saveMap("data/" + to_string(mapNum) + "map.json", obstacleHash);
				ofstream f("data/" + to_string(mapNum) + "paths.json", std::ofstream::app);
				// f.seekp(2, ios_base::end);
				// f << "\b";
				f << "]";
				// f.seekp(f.tellp() - 1);
				// f.write("]", 1);
				// f.close();
				mapNum++;
			}

			delete planner;
			obstacleRects.clear();
			generateObstacleRects(width, height, 10, obstacleRects, OBSTACLE_PADDING);

			/*delete[] obstacleHash;
			obstacleHash = new vector<vector<bool>>(height, vector<bool>(width, false));*/
			obstacleHash = vector<vector<bool>>(height, vector<bool>(width, false));
			/*for (int r = 0; r < obstacleHash->size(); r++) {
			    for (int c = 0; c < (*obstacleHash)[r].size(); c++) {
			        (*obstacleHash)[r][c] = false;
			    }
			}*/
			generateObstacleHash(obstacleRects, obstacleHash);

			// delete planner;
			planner = new AStar(&obstacleHash, &obstacleRects, width, height, usePseudoRandom);
			appendPath("data/" + to_string(mapNum) + "paths.json", planner->root->coord, planner->endNode->coord, planner->bestPath);
			// printf("created planner\n");
		}

		auto currentTime = glfwGetTime();
		if (currentTime - lastFrame >= frameInterval) {
			lastFrame = currentTime;
			// printf("replans: %d\n", replanCount * 30);
			// replanCount = 0;

			glClear(GL_COLOR_BUFFER_BIT);
			display(planner->root, planner->endNode, planner->bestPath, *planner->obstacleRects);
			glfwSwapBuffers(window);
			glfwPollEvents();
		} else {  // if (replanFrequency != -1 && currentTime - lastReplan >= replanInterval) {
			// lastReplan = currentTime;
			planner->randomStart();
			planner->randomReplan();
			replanCount++;

			appendPath("data/" + to_string(mapNum) + "paths.json", planner->root->coord, planner->endNode->coord, planner->bestPath);
		}
	}
	glfwDestroyWindow(window);
	glfwTerminate();
	exit(EXIT_SUCCESS);
}
