#include <functional>
#include <memory>
#include <set>
#include <vector>
#include <GLFW/glfw3.h>
#include "libs/color.hpp"

static void onError(int error, const char* description) { fputs(description, stderr); }

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

inline void setColor(HSL& hsl) {
	auto rgb = HSLToRGB(hsl);
	glColor3d(rgb.R, rgb.G, rgb.B);
}

inline void drawPoint(Coord point, double radius, HSL hsl) {
	glEnable(GL_POINT_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glPointSize(radius);
	// glColor3d(1.0, 1.0, 0);
	setColor(hsl);

	glBegin(GL_POINTS);
	glVertex2d(point.x, point.y);
	glEnd();
}

inline void drawLine(Coord start, Coord end, int linewidth, HSL hsl) {
	setColor(hsl);
	glLineWidth(linewidth);
	glBegin(GL_LINES);
	glVertex2d(start.x, start.y);
	glVertex2d(end.x, end.y);
	glEnd();
}

inline void drawTree(const std::shared_ptr<Node>& root) {
	for (auto child : root->children) {
		drawLine(root->coord, child->coord, 1, HSL(100, 1, 0.5));
		drawTree(child);
	}
}

inline void drawGraphRecursive(const std::shared_ptr<Node>& node, std::set<std::shared_ptr<Node>>& visited) {
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

		drawLine(node->coord, child->coord, linewidth, *hsl);
		if (visited.find(child) == visited.end()) {
			drawGraphRecursive(child, visited);
		}
	}
}

/*
void drawGraphIterative(set<Node*>& visited, unordered_map<Node*, std::vector<Node*>> visibilityGraph) {
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

void drawGraph(Node* root, unordered_map<Node*, std::vector<Node*>> visibilityGraph) {
    set<Node*> visited;
    // drawGraphRecursive(root, visited);
    drawGraphIterative(visited, visibilityGraph);
}
*/

inline void drawPath(std::deque<Coord>& path, HSL lineHsl, HSL pointHsl) {
	glEnable(GL_LINE_SMOOTH);
	glLineWidth(3);
	// glColor3d(0.0, 1.0, 0.2);
	setColor(lineHsl);

	glBegin(GL_LINE_STRIP);
	for (auto point : path) {
		glVertex2d(point.x, point.y);
	}
	glEnd();

	glDisable(GL_LINE_SMOOTH);

	for (auto point : path) {
		drawPoint(point, 5, pointHsl);
	}
}

inline void drawObstacles(std::vector<std::shared_ptr<Rect>>* obstacleRects, const double OBSTACLE_PADDING, HSL hsl) {
	// glColor3d(0.0, 1.0, 0.5);
	setColor(hsl);
	glBegin(GL_QUADS);
	for (auto obstacle : *obstacleRects) {
		glVertex2d(obstacle->topLeft.x + OBSTACLE_PADDING, obstacle->topLeft.y + OBSTACLE_PADDING);
		glVertex2d(obstacle->bottomRight.x - OBSTACLE_PADDING, obstacle->topLeft.y + OBSTACLE_PADDING);
		glVertex2d(obstacle->bottomRight.x - OBSTACLE_PADDING, obstacle->bottomRight.y - OBSTACLE_PADDING);
		glVertex2d(obstacle->topLeft.x + OBSTACLE_PADDING, obstacle->bottomRight.y - OBSTACLE_PADDING);
	}
	glEnd();
}

inline GLFWwindow* initWindow(bool isFullscreen, int monitorNum, int& width, int& height) {
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

	return window;
}

inline void displayLoop(GLFWwindow* window, double frameRate, std::function<void()> display, std::function<void()> remainder) {
	auto lastFrame = glfwGetTime();
	auto frameInterval = 1.0 / frameRate;

	while (!glfwWindowShouldClose(window)) {
		auto currentTime = glfwGetTime();
		if (currentTime - lastFrame >= frameInterval) {
			lastFrame = currentTime;

			glClear(GL_COLOR_BUFFER_BIT);

			display();

			glfwSwapBuffers(window);
			glfwPollEvents();
			// printf("drawing\n");
		} else {
			remainder();
		}
	}

	glfwDestroyWindow(window);
	glfwTerminate();
	exit(EXIT_SUCCESS);
}