#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include "OnlineFmtStar.hpp"
#include "utils.hpp"

using namespace std;

static void error_callback(int error, const char *description) { fputs(description, stderr); }

static void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) glfwSetWindowShouldClose(window, GL_TRUE);
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

	glBegin(GL_POINTS);
	glColor3d(1.0, 1.0, 0);
	glVertex2d(point.x(), point.y());
	glEnd();
}

void drawLine(Coord start, Coord end) {
	glColor3d(0, 1.0, 1.0);
	glLineWidth(1);
	glBegin(GL_LINES);
	glVertex2d(start.x(), start.y());
	glVertex2d(end.x(), end.y());
	glEnd();
}

void drawTree(Node &root) {
	for (auto child : root.children) {
		drawLine(root.coord, child->coord);
		drawTree(*child);
	}
}

void drawObstacles(vector<Rect*>* obstacleRects) {
  glBegin(GL_QUADS);
  glColor3d(1.0, 0, 1.0);
  for (auto obstacle : *obstacleRects) {
    glVertex2d(obstacle->topLeft.x(), obstacle->topLeft.y());
    glVertex2d(obstacle->bottomRight.x(), obstacle->topLeft.y());
    glVertex2d(obstacle->bottomRight.x(), obstacle->bottomRight.y());
    glVertex2d(obstacle->topLeft.x(), obstacle->bottomRight.y());
  }
  glEnd();
}

void display(Coord startPoint, Coord endPoint, Node &root, vector<Rect*>* obstacleRects) {
  drawObstacles(obstacleRects);
  drawTree(root);

	drawPoint(startPoint, 20);
	drawPoint(endPoint, 20);
}

int main(void) {

	int width, height = 700;
	bool isFullscreen = true;
	int monitorNum = 0;


	GLFWwindow *window;
	glfwSetErrorCallback(error_callback);
	if (!glfwInit()) exit(EXIT_FAILURE);

	GLFWmonitor* monitor;
	if (isFullscreen) {
		int count;
		printf("getting monitors\n");
		GLFWmonitor** monitors = glfwGetMonitors(&count);
		printf("got %d monitors\n", count);
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
	glfwSetKeyCallback(window, key_callback);

	float ratio;
	//int width, height;
	//glfwGetFramebufferSize(window, &width, &height);
	ratio = width / (float)height;
	initDisplay(width, height, ratio);

	vector<Rect*> obstacleRects;
	generateObstacleRects(width, height, 10, obstacleRects);

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

	auto planner = OnlineFmtStar(&obstacleHash, &obstacleRects, 6, width, height);

	auto lastTime = glfwGetTime();
	auto interval = 1.0 / 60.0;

	while (!glfwWindowShouldClose(window)) {
		auto currentTime = glfwGetTime();
		if (currentTime - lastTime >= interval) {
			lastTime = currentTime;
			glClear(GL_COLOR_BUFFER_BIT);

			display(planner.startPoint, planner.endPoint, planner.root, planner.obstacleRects);

			glfwSwapBuffers(window);
			glfwPollEvents();
		} else {
			planner.sample();
		}
	}
	glfwDestroyWindow(window);
	glfwTerminate();
	exit(EXIT_SUCCESS);
}
