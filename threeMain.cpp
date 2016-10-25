//#include <GL/glut.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include "OnlineFmtStar.hpp"
#include "OnlineRrtStar.hpp"
#include "cxxopts.hpp"
#include "utils.hpp"

using namespace std;

struct Camera {
	Coord3 pos = Coord3(0,0,0);
	Coord3 rot = Coord3(0,0,0);
} camera;

struct Moves {
	double uavX = 0.0;
	double uavY = 0.0;
	double endX = 0.0;
	double endY = 0.0;
	Camera cam;
};

const double PI = 3.141592654;


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
		case GLFW_KEY_R:
			if (action != GLFW_RELEASE) {
				currentMoves->cam.pos.sety(MOVERATE);
			} else {
				currentMoves->cam.pos.sety(0);
			}
			break;
		case GLFW_KEY_F:
			if (action != GLFW_RELEASE) {
				currentMoves->cam.pos.sety(-MOVERATE);
			} else {
				currentMoves->cam.pos.sety(0);
			}
			break;
		case GLFW_KEY_W:
			if (action != GLFW_RELEASE) {
				currentMoves->cam.pos.setx(-sin(camera.rot.y()*PI/180) * MOVERATE);
				currentMoves->cam.pos.setz(cos(camera.rot.y()*PI/180) * MOVERATE);
			} else {
				currentMoves->cam.pos.setx(0);
				currentMoves->cam.pos.setz(0);
			}
			break;
		case GLFW_KEY_S:
			if (action != GLFW_RELEASE) {
				currentMoves->cam.pos.setx(sin(camera.rot.y()*PI/180) * MOVERATE);
				currentMoves->cam.pos.setz(-cos(camera.rot.y()*PI/180) * MOVERATE);
			} else {
				currentMoves->cam.pos.setx(0);
				currentMoves->cam.pos.setz(0);
			}
			break;
		case GLFW_KEY_A:
			if (action != GLFW_RELEASE) {
				currentMoves->cam.pos.setx(cos(camera.rot.y()*PI/180) * MOVERATE);
				currentMoves->cam.pos.setz(sin(camera.rot.y()*PI/180) * MOVERATE);
			} else {
				currentMoves->cam.pos.setx(0);
				currentMoves->cam.pos.setz(0);
			}
			break;
		case GLFW_KEY_D:
			if (action != GLFW_RELEASE) {
				currentMoves->cam.pos.setx(-cos(camera.rot.y()*PI/180) * MOVERATE);
				currentMoves->cam.pos.setz(-sin(camera.rot.y()*PI/180) * MOVERATE);
			} else {
				currentMoves->cam.pos.setx(0);
				currentMoves->cam.pos.setz(0);
			}
			break;
		case GLFW_KEY_Q:
			if (action != GLFW_RELEASE) {
				currentMoves->cam.rot.sety(-MOVERATE);
			} else {
				currentMoves->cam.rot.sety(0);
			}
			break;
		case GLFW_KEY_E:
			if (action != GLFW_RELEASE) {
				currentMoves->cam.rot.sety(MOVERATE);
			} else {
				currentMoves->cam.rot.sety(0);
			}
			break;
	}

	// printf("key: %d\n, (%.2f, %.2f)", key, currentMoves->uavX, currentMoves->uavY);
}

void perspectiveGL(GLdouble fovY, GLdouble aspect, GLdouble zNear, GLdouble zFar) {
	const GLdouble pi = 3.1415926535897932384626433832795;
	GLdouble fW, fH;

	// fH = tan( (fovY / 2) / 180 * pi ) * zNear;
	fH = tan(fovY / 360 * pi) * zNear;
	fW = fH * aspect;

	glFrustum(-fW, fW, -fH, fH, zNear, zFar);
}

void initDisplay(int width, int height, float ratio) {
	glViewport(0, 0, width, height);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	// glOrtho(0, width, 0, height, -1.f, 1.f);
	perspectiveGL(75, width / height, 1, 10000);
	glScalef(1, -1, 1);
	// glTranslatef(-width/2, -height/2, -1600);
	// glEnable(GL_BLEND);
	// glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glfwSwapInterval(0);

	GLfloat light_ambientColor[] = {0.01, 0.01, 0.01, 1};
	GLfloat light_color[] = {1.0, 1.0, 1.0, 1.0};

	// GLfloat light_position[] = {400.0, 200.0, 0.0, 1.0};
	// glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_color);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_color);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_color);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_ambientColor);
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
	// glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glShadeModel(GL_FLAT);

	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	// glDisable(GL_DEPTH_TEST);
}

void drawPoint(Coord point, double radius) {
	glPushMatrix();
	glTranslated(point.x(), point.y(), 0);
	glColor3d(1.0, 1.0, 0);
	gluSphere(gluNewQuadric(), radius, 20, 20);
	glPopMatrix();
	/*glEnable(GL_POINT_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glPointSize(radius);
	glColor3d(1.0, 1.0, 0);

	glBegin(GL_POINTS);
	glVertex2d(point.x(), point.y());
	glEnd();*/
}

void drawLine(Coord start, Coord end) {
	glColor3d(0.5, 0.0, 1.0);
	// GLfloat mat_ambient[] = { 0.7, 0.7, 0.7, 1.0 };
	// GLfloat mat_diffuse[] = { .5, 0, 1.0, 1.0 };
	// glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	// glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
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
	// GLfloat mat_ambient[] = { 0.7, 0.7, 0.7, 1.0 };
	// GLfloat mat_diffuse[] = { 0, 1.0, 0.2, 1.0 };
	// glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	// glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);

	glBegin(GL_LINE_STRIP);
	for (auto point : path) {
		glVertex2d(point.x(), point.y());
	}
	glEnd();

	glDisable(GL_LINE_SMOOTH);
}

void drawObstacles(vector<Rect*>* obstacleRects) {
	glColor3d(0.0, .3, .3);
	// GLfloat mat_ambient[] = { 0, 0.7, 0.7, 1.0 };
	// GLfloat mat_diffuse[] = { 0, 0.7, 0.7, 1.0 };
	// glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	// glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glBegin(GL_QUADS);
	for (auto obstacle : *obstacleRects) {
		// Top
		// glNormal3i(0,0,-1);
		glVertex3d(obstacle->topLeft.x(), obstacle->bottomRight.y(), 100);
		glVertex3d(obstacle->bottomRight.x(), obstacle->bottomRight.y(), 100);
		glVertex3d(obstacle->bottomRight.x(), obstacle->topLeft.y(), 100);
		glVertex3d(obstacle->topLeft.x(), obstacle->topLeft.y(), 100);

		// Bottom
		// glNormal3i(0,0,1);
		glVertex3d(obstacle->topLeft.x(), obstacle->topLeft.y(), 0);
		glVertex3d(obstacle->bottomRight.x(), obstacle->topLeft.y(), 0);
		glVertex3d(obstacle->bottomRight.x(), obstacle->bottomRight.y(), 0);
		glVertex3d(obstacle->topLeft.x(), obstacle->bottomRight.y(), 0);

		// Back
		// glNormal3i(1,0,0);
		glVertex3d(obstacle->bottomRight.x(), obstacle->bottomRight.y(), 100);
		glVertex3d(obstacle->topLeft.x(), obstacle->bottomRight.y(), 100);
		glVertex3d(obstacle->topLeft.x(), obstacle->bottomRight.y(), 0);
		glVertex3d(obstacle->bottomRight.x(), obstacle->bottomRight.y(), 0);

		// Front
		glVertex3d(obstacle->topLeft.x(), obstacle->topLeft.y(), 100);
		glVertex3d(obstacle->bottomRight.x(), obstacle->topLeft.y(), 100);
		glVertex3d(obstacle->bottomRight.x(), obstacle->topLeft.y(), 0);
		glVertex3d(obstacle->topLeft.x(), obstacle->topLeft.y(), 0);

		// Right
		glVertex3d(obstacle->bottomRight.x(), obstacle->topLeft.y(), 100);
		glVertex3d(obstacle->bottomRight.x(), obstacle->bottomRight.y(), 100);
		glVertex3d(obstacle->bottomRight.x(), obstacle->bottomRight.y(), 0);
		glVertex3d(obstacle->bottomRight.x(), obstacle->topLeft.y(), 0);

		// Left
		glVertex3d(obstacle->topLeft.x(), obstacle->topLeft.y(), 0);
		glVertex3d(obstacle->topLeft.x(), obstacle->bottomRight.y(), 0);
		glVertex3d(obstacle->topLeft.x(), obstacle->bottomRight.y(), 100);
		glVertex3d(obstacle->topLeft.x(), obstacle->topLeft.y(), 100);
	}
	glEnd();
}

void placeCamera(Camera cam) {
	// glMatrixMode(GL_PROJECTION);
	// glLoadIdentity();
	// // glOrtho(0, width, 0, height, -1.f, 1.f);
	// perspectiveGL(75,width/height,1, 10000);
	// glScalef(1, -1, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	GLfloat light_position[] = {500.0, 200.0, 120.0, 1.0};
	// rotate(cam.rot.x(), cos(radians(cam.rot.y())),0,sin(radians(cam.rot.y()))) //Translate to c++ later
	glRotatef(cam.rot.y(), 0, 1, 0);
	glTranslatef(cam.pos.x(), cam.pos.y(), cam.pos.z());
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	// glMatrixMode(GL_PROJECTION);
}

void display(Node* root, Node* endNode, deque<Coord>& bestPath, vector<Rect*>* obstacleRects, Camera& cam) {
	glClear(GL_DEPTH_BUFFER_BIT);
	placeCamera(cam);

	drawObstacles(obstacleRects);

	glDisable(GL_LIGHTING);
	drawTree(root);
	glEnable(GL_LIGHTING);

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

	// clang-format off
	cxxopts::Options options("OnlineRRT*", "A cool program for cool things");
	options.add_options()
		("f,fullscreen", "Enable Fullscreen", cxxopts::value(isFullscreen))
		("m,monitor", "Set Monitor Number", cxxopts::value(monitorNum))
    ("fmt", "Use FMT*", cxxopts::value(useFmt));
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

	Planner* planner;
	if (useFmt) {
		planner = new OnlineFmtStar(&obstacleHash, &obstacleRects, 6, width, height, false);
	} else {
		planner = new OnlineRrtStar(&obstacleHash, &obstacleRects, 6, width, height, false);
	}

	auto lastTime = glfwGetTime();
	auto interval = 1.0 / 60.0;
	///*
	// auto iterations = 0;
	// auto frames = 0;
	// double averageIterations = 0;
	//*/
	// double averageRenderTime = 0;
	// double beginFrame, endFrame, renderTime;
	Moves currentMoves;
	glfwSetWindowUserPointer(window, &currentMoves);

	// Camera cam;
	camera.pos = Coord3(-width/2, -height/2, -1600);
	// glfwSetWindowUserPointer(window, &cam);

	while (!glfwWindowShouldClose(window)) {
		auto currentTime = glfwGetTime();
		if (currentTime - lastTime >= interval) {
			lastTime = currentTime;
			/*
			averageIterations = (averageIterations * frames + iterations) / (frames + 1.0);
			printf("i: %.2f\n", averageIterations);
			frames += 1;
			iterations = 0;
			//*/
			// beginFrame = glfwGetTime();
			camera.pos.setx(camera.pos.x()+currentMoves.cam.pos.x());
			camera.pos.sety(camera.pos.y()+currentMoves.cam.pos.y());
			camera.pos.setz(camera.pos.z()+currentMoves.cam.pos.z());
			camera.rot.setx(camera.rot.x()+currentMoves.cam.rot.x());
			camera.rot.sety(camera.rot.y()+currentMoves.cam.rot.y());
			camera.rot.setz(camera.rot.z()+currentMoves.cam.rot.z());

			glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
			display(planner->root, planner->endNode, planner->bestPath, planner->obstacleRects, camera);

			glfwSwapBuffers(window);
			glfwPollEvents();

			// endFrame = glfwGetTime();
			// renderTime = endFrame-beginFrame;
			// // averageRenderTime = (averageRenderTime*4.0 + renderTime) / (5.0);
			// printf("t: %.6f\n", renderTime);
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
