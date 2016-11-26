#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <set>
#include "color.hpp"
#include "cxxopts.hpp"
#include "planning/AStar.hpp"
#include "planning/OnlineFmtStar.hpp"
#include "planning/OnlineRrtStar.hpp"
#include "planning/utils.hpp"

using namespace std;

struct Moves {
	double uavX = 0.0;
	double uavY = 0.0;
	double endX = 0.0;
	double endY = 0.0;
};

const double MOVERATE = 3;
const double OBSTACLE_PADDING = 5;

bool planAgain = false;

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
		case GLFW_KEY_P:
			if (action == GLFW_RELEASE) {
				planAgain = true;
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

void drawLine(Coord start, Coord end, HSL hsl, int linewidth) {
	auto rgb = HSLToRGB(hsl);
	glColor3d(rgb.R, rgb.G, rgb.B);
	glLineWidth(linewidth);
	glBegin(GL_LINES);
	glVertex2d(start.x(), start.y());
	glVertex2d(end.x(), end.y());
	glEnd();
}

void drawTree(const shared_ptr<Node>& root) {
	for (auto child : root->children) {
		drawLine(root->coord, child->coord, HSL(100, 1, 0.5), 1);
		drawTree(child);
	}
}

void drawGraphRecursive(const shared_ptr<Node>& node, set<shared_ptr<Node>>& visited) {
	visited.insert(node);
	for (auto child : node->children) {
		HSL* hsl;
		int linewidth = 3;
		switch (child->status) {
			case Status::Unvisited:
				hsl = new HSL(100, 1, 0.5);
				linewidth = 1;
				break;
			case Status::Open:
				hsl = new HSL(50, 1, 0.5);
				break;
			case Status::Closed:
				hsl = new HSL(250, 1, 0.5);
				break;
			default:
				hsl = new HSL(360, 1, 0.5);
		}

		drawLine(node->coord, child->coord, *hsl, linewidth);
		if (visited.find(child) == visited.end()) {
			drawGraphRecursive(child, visited);
		}
	}
}

/*
void drawGraphIterative(set<Node*>& visited, unordered_map<Node*, vector<Node*>> visibilityGraph) {
    for (auto kv_pair : visibilityGraph) {
        auto parent = kv_pair.first;

        if (visited.find(parent) != visited.end()) {
            continue;
        }

        for (auto child : kv_pair.second) {
            HSL* hsl;
            int linewidth = 3;
            switch (child->status) {
                case Status::Unvisited:
                    hsl = new HSL(100, 1, 0.5);
                    linewidth = 1;
                    break;
                case Status::Open:
                    hsl = new HSL(50, 1, 0.5);
                    linewidth = 1;
                    break;
                case Status::Closed:
                    hsl = new HSL(250, 1, 0.5);
                    break;
                default:
                    hsl = new HSL(360, 1, 0.5);
            }

            if (child->status == Status::Closed) {
                drawLine(parent->coord, child->coord, *hsl, linewidth);
            }
        }
    }
}

void drawGraph(Node* root, unordered_map<Node*, vector<Node*>> visibilityGraph) {
    set<Node*> visited;
    // drawGraphRecursive(root, visited);
    drawGraphIterative(visited, visibilityGraph);
}
*/

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

void drawObstacles(vector<shared_ptr<Rect>>* obstacleRects) {
	glColor3d(0.0, 1.0, 1.0);
	glBegin(GL_QUADS);
	for (auto obstacle : *obstacleRects) {
		glVertex2d(obstacle->topLeft.x() + OBSTACLE_PADDING, obstacle->topLeft.y() + OBSTACLE_PADDING);
		glVertex2d(obstacle->bottomRight.x() - OBSTACLE_PADDING, obstacle->topLeft.y() + OBSTACLE_PADDING);
		glVertex2d(obstacle->bottomRight.x() - OBSTACLE_PADDING, obstacle->bottomRight.y() - OBSTACLE_PADDING);
		glVertex2d(obstacle->topLeft.x() + OBSTACLE_PADDING, obstacle->bottomRight.y() - OBSTACLE_PADDING);
	}
	glEnd();
}

void display(const shared_ptr<Node> root, const shared_ptr<Node>& endNode, deque<Coord>& bestPath, vector<shared_ptr<Rect>>* obstacleRects) {
	drawObstacles(obstacleRects);
	// drawTree(root);
	// drawGraph(root, visibilityGraph);

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
	double replanFrequency = -1;

	// clang-format off
	cxxopts::Options options("OnlineRRT*", "A cool program for cool things");
	options.add_options()
		("f,fullscreen", "Enable Fullscreen", cxxopts::value(isFullscreen))
		("m,monitor", "Set Monitor Number", cxxopts::value(monitorNum))
		("fmt", "Use FMT*", cxxopts::value(useFmt))
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

	float ratio;
	// int width, height;
	// glfwGetFramebufferSize(window, &width, &height);
	ratio = width / (float)height;
	initDisplay(width, height, ratio);

	vector<shared_ptr<Rect>> obstacleRects;
	generateObstacleRects(width, height, 10, obstacleRects, OBSTACLE_PADDING);

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

	/*SamplingPlanner* planner;
	if (useFmt) {
	    planner = new OnlineFmtStar(&obstacleHash, &obstacleRects, 6, width, height, usePseudoRandom);
	} else {
	    planner = new OnlineRrtStar(&obstacleHash, &obstacleRects, 6, width, height, usePseudoRandom);
	}*/
	AStar* planner = new AStar(&obstacleHash, &obstacleRects, width, height, usePseudoRandom);

	auto lastFrame = glfwGetTime();
	auto frameInterval = 1.0 / 30.0;

	auto lastReplan = glfwGetTime();
	auto replanInterval = 1.0 / replanFrequency;
	/*auto iterations = 0;
	auto frames = 0;
	double averageIterations = 0;*/
	printf("%.2f\n", replanInterval);

	Moves currentMoves;
	glfwSetWindowUserPointer(window, &currentMoves);

	while (!glfwWindowShouldClose(window)) {
		auto currentTime = glfwGetTime();
		if (currentTime - lastFrame >= frameInterval) {
			lastFrame = currentTime;
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

			planner->followPath();
			planner->moveStart(currentMoves.uavX, currentMoves.uavY);
		} else if (replanFrequency != -1 && currentTime - lastReplan >= replanInterval) {
			lastReplan = currentTime;
			planner->randomReplan();
			// printf("replan: %.5f\n", glfwGetTime() - lastReplan);
		} else {
			// iterations++;
			// planner->sample();
		}

		if (planAgain) {
			// planner->plan();
			planAgain = false;
		}
	}
	glfwDestroyWindow(window);
	glfwTerminate();
	exit(EXIT_SUCCESS);
}
