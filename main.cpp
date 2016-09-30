#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <boost/program_options.hpp>
#include "OnlineFmtStar.hpp"
#include "utils.hpp"

using namespace std;
using namespace boost::program_options;

struct Moves {
	double uavX = 0.0;
	double uavY = 0.0;
	double endX = 0.0;
	double endY = 0.0;
};

const double MOVERATE = 0.003;

static void onError(int error, const char* description) { fputs(description, stderr); }

static void onKey(GLFWwindow* window, int key, int scancode, int action, int mods) {
	Moves* currentMoves = static_cast<Moves*>(glfwGetWindowUserPointer(window));

	switch (key) {
		case GLFW_KEY_ESCAPE:
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
		case GLFW_KEY_UP:
			if (action != GLFW_RELEASE) {
				currentMoves->uavY = MOVERATE;
			} else {
				currentMoves->uavY = 0;
			}
			break;
		case GLFW_KEY_DOWN:
			if (action != GLFW_RELEASE) {
				currentMoves->uavY = -MOVERATE;
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
	// glViewport(0, 0, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, 1, 0, 1, -1.f, 1.f);
	// glScalef(1, -1, 1);
	// glTranslatef(0, -1, 0);
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

int main(int argc, const char* argv[]) {
	try {
		options_description desc{"Options"};
		desc.add_options()("width, w", value<int>()->default_value(700), "Width")("height, h", value<int>()->default_value(700), "Height")(
		    "full, f", "Fullscreen")("monitor, m", value<int>()->default_value(0), "Monitor");

		variables_map vm;
		store(parse_command_line(argc, argv, desc), vm);
		notify(vm);

		int width = vm["width"].as<int>();
		int height = vm["height"].as<int>();
		// int monitorNum = vm["monitor"];

		GLFWwindow* window;
		glfwSetErrorCallback(onError);
		if (!glfwInit()) exit(EXIT_FAILURE);

		GLFWmonitor* monitor;
		if (vm.count("full")) {
			int count;
			GLFWmonitor** monitors = glfwGetMonitors(&count);
			monitor = monitors[vm["monitor"].as<int>()];
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
		// glfwSetKeyCallback(window, key_callback);

		float ratio;
		// int width, height;
		// glfwGetFramebufferSize(window, &width, &height);
		ratio = width / (float)height;
		initDisplay(width, height, ratio);

		double obstacleHashHeight = 100;
		double obstacleHashWidth = obstacleHashHeight * ratio;

		vector<Rect*> obstacleRects;
		generateObstacleRects(1.0, 1.0, 10, obstacleRects);

		vector<vector<bool>> obstacleHash(obstacleHashHeight, vector<bool>(obstacleHashWidth, false));
		generateObstacleHash(obstacleRects, obstacleHash, obstacleHashWidth, obstacleHashHeight);

		for (auto row : obstacleHash) {
			for (auto value : row) {
				printf("%d", value ? 1 : 0);
			}
			printf("\n");
		}

		auto planner = OnlineFmtStar(&obstacleHash, &obstacleRects, 0.2, 1.0, 1.0, obstacleHashWidth, obstacleHashHeight);

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

				display(planner.root, planner.endNode, planner.bestPath, planner.obstacleRects);

				glfwSwapBuffers(window);
				glfwPollEvents();

				planner.moveStart(currentMoves.uavX, currentMoves.uavY);
			} else {
				// iterations++;
				planner.sample();
			}
		}
		glfwDestroyWindow(window);
		glfwTerminate();
		exit(EXIT_SUCCESS);
	} catch (const error& ex) {
		cerr << ex.what() << endl;
	}
}
