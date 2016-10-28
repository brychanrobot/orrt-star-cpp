#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include "OnlineFmtStar.hpp"
#include "OnlineRrtStar.hpp"
#include "cxxopts.hpp"
#include "utils.hpp"

using namespace std;

struct Moves {
	double uavX = 0.0;
	double uavY = 0.0;
	double endX = 0.0;
	double endY = 0.0;
};

const double MOVERATE = 3;

static void onError(int error, const char* description) { fputs(description, stderr); }

static void onKey(GLFWwindow* window, int key, int scancode, int action, int mods) {
	Moves* currentMoves = static_cast<Moves*>(glfwGetWindowUserPointer(window));

	switch (key) {
		case GLFW_KEY_ESCAPE:
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
		case GLFW_KEY_UP:
			if (action != GLFW_RELEASE) {
				currentMoves->uavY = -MOVERATE;
			} else {
				currentMoves->uavY = 0;
			}
			break;
		case GLFW_KEY_DOWN:
			if (action != GLFW_RELEASE) {
				currentMoves->uavY = MOVERATE;
			} else {
				currentMoves->uavY = 0;
			}
			break;
		case GLFW_KEY_LEFT:
			if (action != GLFW_RELEASE) {
				currentMoves->uavX = -MOVERATE;
			} else {
				currentMoves->uavX = 0;
			}
			break;
		case GLFW_KEY_RIGHT:
			if (action != GLFW_RELEASE) {
				currentMoves->uavX = MOVERATE;
			} else {
				currentMoves->uavX = 0;
			}
			break;
	}

	// printf("key: %d\n, (%.2f, %.2f)", key, currentMoves->uavX, currentMoves->uavY);
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

void drawLine(Coord start, Coord end) {
	glColor3d(0.5, 0.0, 1.0);
	glLineWidth(1);
	glBegin(GL_LINES);
	glVertex2d(start.x(), start.y());
	glVertex2d(end.x(), end.y());
	glEnd();
}

void drawTree(Node* root) {
	for (auto child : root->children) {
		drawLine(root->coord, child->coord);
		drawTree(child);
	}
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

void drawObstacles(vector<Rect*>* obstacleRects) {
	glColor3d(0.0, 1.0, 1.0);
	glBegin(GL_QUADS);
	for (auto obstacle : *obstacleRects) {
		glVertex2d(obstacle->topLeft.x(), obstacle->topLeft.y());
		glVertex2d(obstacle->bottomRight.x(), obstacle->topLeft.y());
		glVertex2d(obstacle->bottomRight.x(), obstacle->bottomRight.y());
		glVertex2d(obstacle->topLeft.x(), obstacle->bottomRight.y());
	}
	glEnd();
}

void display(Node* root, Node* endNode, deque<Coord>& bestPath, vector<Rect*>* obstacleRects) {
	drawObstacles(obstacleRects);
	drawTree(root);

	drawPath(bestPath);

	drawPoint(root->coord, 20);
	drawPoint(endNode->coord, 20);
}

int main(int argc, char* argv[]) {
	int width = 700;
	int height = 700;
	bool isFullscreen = false;
	int monitorNum = 0;
	bool useFmt = false;
	bool usePseudoRandom = false;

	// clang-format off
	cxxopts::Options options("OnlineRRT*", "A cool program for cool things");
	options.add_options()
		("f,fullscreen", "Enable Fullscreen", cxxopts::value(isFullscreen))
		("m,monitor", "Set Monitor Number", cxxopts::value(monitorNum))
		("fmt", "Use FMT*", cxxopts::value(useFmt))
		("p,pr", "Use pseudo-random numbers", cxxopts::value(usePseudoRandom));
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

	float ratio;
	// int width, height;
	// glfwGetFramebufferSize(window, &width, &height);
	ratio = width / (float)height;
	initDisplay(width, height, ratio);

	vector<Rect*> obstacleRects;
	generateObstacleRects(width, height, 0, 10, obstacleRects);

	vector<vector<bool>> obstacleHash(height, vector<bool>(width, false));
	generateObstacleHash(obstacleRects, obstacleHash);

	/*
	for (auto row: obstacleHash) {
	    for (auto value: row) {
	        printf("%d", value ? 1 : 0);
	    }
	    printf("\n");
	}
	*/

	Planner* planner;
	if (useFmt) {
		planner = new OnlineFmtStar(&obstacleHash, &obstacleRects, 6, width, height, 500, usePseudoRandom);
	} else {
		planner = new OnlineRrtStar(&obstacleHash, &obstacleRects, 6, width, height, 500, usePseudoRandom);
	}

	auto lastTime = glfwGetTime();
	auto interval = 1.0 / 60.0;
	/*auto iterations = 0;
	auto frames = 0;
	double averageIterations = 0;*/

	Moves currentMoves;
	glfwSetWindowUserPointer(window, &currentMoves);

	while (!glfwWindowShouldClose(window)) {
		auto currentTime = glfwGetTime();
		if (currentTime - lastTime >= interval) {
			lastTime = currentTime;
			/*
			averageIterations = (averageIterations * frames + iterations) / (frames + 1.0);
			printf("i: %.2f\n", averageIterations);
			frames += 1;
			iterations = 0;
			*/

			glClear(GL_COLOR_BUFFER_BIT);

			display(planner->root, planner->endNode, planner->bestPath, planner->obstacleRects);

			glfwSwapBuffers(window);
			glfwPollEvents();

			planner->moveStart(currentMoves.uavX, currentMoves.uavY);
		} else {
			// iterations++;
			planner->sample();
		}
	}
	glfwDestroyWindow(window);
	glfwTerminate();
	exit(EXIT_SUCCESS);
}
