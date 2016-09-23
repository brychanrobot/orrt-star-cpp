#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <stdio.h>
#include "OnlineFmtStar.hpp"
#include "utils.hpp"

using namespace std;

static void error_callback(int error, const char* description)
{
    fputs(description, stderr);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}

void initDisplay(int width, int height, float ratio)
{
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

void display(Coord startPoint, Coord endPoint)
{
  drawPoint(startPoint, 20);
  drawPoint(endPoint, 20);

  //printf("point: %f, %f\n", startPoint.x, startPoint.y);
  //printf("point: %f, %f\n", endPoint.x, endPoint.y);

  /*
  glRotatef((float) glfwGetTime() * 50.f, 0.f, 0.f, 1.f);
  glBegin(GL_TRIANGLES);
  glColor3f(1.f, 0.f, 0.f);
  glVertex3f(-0.6f, -0.4f, 0.f);
  glColor3f(0.f, 1.f, 0.f);
  glVertex3f(0.6f, -0.4f, 0.f);
  glColor3f(0.f, 0.f, 1.f);
  glVertex3f(0.f, 0.6f, 0.f);
  glEnd();
  */
}

void generateObstacles(vector<vector<bool>>* obstacleHash, vector<Rect>* obstacleRects, int width, int height)
{

}

int main(void)
{
    GLFWwindow* window;
    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
        exit(EXIT_FAILURE);
    window = glfwCreateWindow(640, 480, "Simple example", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, key_callback);

    float ratio;
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    ratio = width / (float) height;
    initDisplay(width, height, ratio);

    vector<vector<bool>> obstacleHash(height, vector<bool>(width, false));
    generateObstacles(&obstacleHash, NULL, width, height);

    auto planner = OnlineFmtStar(&obstacleHash, NULL, 30, width, height);

    auto lastTime = glfwGetTime();
    auto interval = 1.0/60.0;

    while (!glfwWindowShouldClose(window))
    {
      auto currentTime = glfwGetTime();
      if(currentTime - lastTime >= interval)
      {
        lastTime = currentTime;
        glClear(GL_COLOR_BUFFER_BIT);

        display(planner.startPoint, planner.endPoint);

        glfwSwapBuffers(window);
        glfwPollEvents();
      }
      else
      {
        planner.sample();
      }
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
}
