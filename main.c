#include <stdio.h>
#include "SDL/SDL.h"
#include <GLES3/gl3.h>
#include "emscripten.h"
#include <math.h>
#include <cglm/cglm.h> 
#include <dwa.h>

#define RESOLUTION_WIDTH  600
#define RESOLUTION_HEIGHT 600
#define M_PI2 2*M_PI

// Fixed size point cloud, no dynamic allocation
typedef struct {
  int size;
  Point points[1000];
} noMemPointCloud;

GLint uniColor;
GLint uniTrans;

GLint pointColor;
GLint pointTrans;

GLuint shaderProgram;
GLuint pointsProgram;

int numberOfSides = 10;
int numberOfVertices;

noMemPointCloud *pointCloud;
PointCloud *tmpPointCloud;
int currentIdx = 0;

mat4 translations[1000];
mat4 trans;

Velocity velocity;

Point goal;
Pose pose;

Config config;
Rect rect;

int drawing = 0;

float pointx;
float pointy;

void update(){
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    switch (event.type) {
    case SDL_QUIT:
      break;
    case SDL_KEYDOWN:
      switch (event.key.keysym.sym) {
      case SDLK_r:
        drawing = 0;
        currentIdx = 0;
        pose.point.x = 0.0;
        pose.point.y = 0.0;
        pose.yaw = 0.0;
        velocity.linearVelocity = 0.0;
        velocity.angularVelocity = 0.0;
        break;
      }
    case SDL_MOUSEBUTTONDOWN:
      drawing = 1;
      break;
    case SDL_MOUSEBUTTONUP:
      drawing = 0;
      break;
    case SDL_MOUSEMOTION:
      pointx = ((float)event.button.x/(float)RESOLUTION_WIDTH-0.5) * 2.0;
      pointy = (-(float)event.button.y/(float)RESOLUTION_HEIGHT+0.5) * 2.0;
      if (drawing && currentIdx < 1000) {
        pointCloud->points[currentIdx].x = pointx*10;
        pointCloud->points[currentIdx].y = pointy*10;
        currentIdx++;
      } else {
        goal.x = pointx*10;
        goal.y = pointy*10;
      }
      break;
    }
  }

  glClear(GL_COLOR_BUFFER_BIT);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

  glUseProgram(shaderProgram);
  // Draw Robot
  glm_mat4_identity(trans);
  glm_translate(trans, (vec3){pose.point.x/10, pose.point.y/10, 0.0f}); // Position
  glm_rotate(trans, pose.yaw, (vec3){0.0f, 0.0f, 1.0f}); // Yaw
  glUniformMatrix4fv(uniTrans, 1, GL_FALSE, trans[0]);
  glUniform3f(uniColor, 0.0f, 0.0f, 1.0f); // Color
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0); // Draw

  // Draw Goal
  glm_mat4_identity(trans);
  glm_translate(trans, (vec3){goal.x/10, goal.y/10, 0.0f});
  glm_scale(trans, (vec3){0.01f, 0.01f, 1.0f});
  glm_rotate(trans, 0, (vec3){0.0f, 0.0f, 1.0f});
  glUniformMatrix4fv(uniTrans, 1, GL_FALSE, trans[0]);
  glUniform3f(uniColor, 0.5f, 0.1f, 0.4f); // Color
  glDrawArrays(GL_TRIANGLE_FAN, 4, numberOfVertices);

  if (currentIdx) {
    tmpPointCloud = createPointCloud(currentIdx);
    memcpy(tmpPointCloud->points, pointCloud->points, currentIdx*sizeof(Point));
    velocity = planning(pose, velocity, goal, tmpPointCloud, config);
    pose = motion(pose, velocity, config.dt);
    freePointCloud(tmpPointCloud);
  }

  // Draw Point Cloud
  glUseProgram(pointsProgram);
  if (currentIdx) {
    for (int i = 0; i < currentIdx; i++) {
      glm_mat4_identity(translations[i]);
      glm_translate(translations[i], (vec3){
          pointCloud->points[i].x/10,
            pointCloud->points[i].y/10,
            0.0f});
      glm_scale(translations[i], (vec3){0.01f, 0.01f, 1.0f});
      glm_rotate(translations[i], 0, (vec3){0.0f, 0.0f, 1.0f});
      glUniform3f(pointColor, 1.0f, 1.0f, 1.0f); // Color
    }
    glUniformMatrix4fv(pointTrans, currentIdx, GL_FALSE, translations[0][0]);
    glDrawArraysInstanced(GL_TRIANGLE_FAN, 4, numberOfVertices, currentIdx);
  }
  SDL_GL_SwapBuffers();
};

int main(){

  numberOfVertices = numberOfSides + 2;

  // Obstacles
  pointCloud->size = 1000;

  // Robot Velocity
  velocity.linearVelocity = 0.0;
  velocity.angularVelocity = 0.0;

  // Robot Position
  pose.point.x = 0.0;
  pose.point.y = 0.0;
  pose.yaw = 0.0;

  // Robot Base
  rect.xmin = -1.0;
  rect.ymin = -0.8;
  rect.xmax = +1.0;
  rect.ymax = +0.8;

  // DWA Configuration
  config.maxSpeed = 2.0;
  config.minSpeed = 0.0;
  config.maxYawrate = 60.0 * M_PI / 180.0;
  config.maxAccel = 15.0;
  config.maxdYawrate = 110.0 * M_PI / 180.0;
  config.velocityResolution = 0.1;
  config.yawrateResolution = 1.0 * M_PI / 180.0;
  config.dt = 0.1;
  config.predictTime = 1.0;
  config.heading = 0.15;
  config.clearance = 1.0;
  config.velocity = 1.0;
  config.base = rect;

  // Initialize Goal to 0
  goal.x = 0.0;
  goal.y = 0.0;

  // Initialize SDL
  if (SDL_Init(SDL_INIT_EVERYTHING) == -1) return 1;
  SDL_WM_SetCaption("test", NULL);
  SDL_SetVideoMode(RESOLUTION_WIDTH, RESOLUTION_HEIGHT, 0, SDL_OPENGL);

  float vertices[8 + numberOfVertices*2];

  // Rectangle Robot Vertices
  vertices[0] = rect.xmin/10;
  vertices[1] = rect.ymax/10;

  vertices[2] = rect.xmax/10;
  vertices[3] = rect.ymax/10;

  vertices[4] = rect.xmax/10;
  vertices[5] = rect.ymin/10;

  vertices[6] = rect.xmin/10;
  vertices[7] = rect.ymin/10;
  GLuint elements[] = {0, 1, 2, 2, 3, 0};

  // Circle Vertices
  vertices[8] = 0.0f;
  vertices[9] = 0.0f;
  int circle_idx = 1;
  for (int i = 10; i < numberOfVertices*2+10; i+=2) {
    vertices[i] = (cos(circle_idx *  M_PI2 / numberOfSides));
    vertices[i+1] = (sin(circle_idx * M_PI2 / numberOfSides));
    circle_idx++;
  }

  // OpenGL Boiler Plate
  GLuint vbo;
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  GLuint ebo;
  glGenBuffers(1, &ebo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(elements), elements, GL_STATIC_DRAW);

  glClear(GL_COLOR_BUFFER_BIT);

  const char* vertexSource =
    "#version 300 es\n"
    "in vec2 position;\n"
    "uniform  mat4 trans;\n"
    "void main() {\n"
    "gl_Position = trans * vec4(position, 0.0, 1.0);\n"
    "}\0";

  const char* circlesSource =
    "#version 300 es\n"
    "in vec2 position;\n"
    "uniform  mat4 trans[1000];\n"
    "void main() {\n"
    "mat4 t = trans[gl_InstanceID];\n"
    "gl_Position = t * vec4(position, 0.0, 1.0);\n"
    "}\0";

  const char* fragmentSource =
    "#version 300 es\n"
    "precision mediump float;\n"
    "uniform vec3 color;\n"
    "out vec4 outColor;\n"
    "void main() {\n"
      "outColor = vec4(color, 1.0);\n"
    "}\0";

  GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertexShader, 1, &vertexSource, NULL);
  glCompileShader(vertexShader);

  GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragmentShader, 1, &fragmentSource, NULL);
  glCompileShader(fragmentShader);

  GLuint circlesShader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(circlesShader, 1, &circlesSource, NULL);
  glCompileShader(circlesShader);

  shaderProgram = glCreateProgram();
  glAttachShader(shaderProgram, vertexShader);
  glAttachShader(shaderProgram, fragmentShader);
  glLinkProgram(shaderProgram);
  glUseProgram(shaderProgram);

  pointsProgram = glCreateProgram();
  glAttachShader(pointsProgram, circlesShader);
  glAttachShader(pointsProgram, fragmentShader);
  glLinkProgram(pointsProgram);
  glUseProgram(pointsProgram);

  uniColor = glGetUniformLocation(shaderProgram, "color");
  uniTrans = glGetUniformLocation(shaderProgram, "trans");

  pointColor = glGetUniformLocation(pointsProgram, "color");
  pointTrans = glGetUniformLocation(pointsProgram, "trans");

  GLint posAttrib = glGetAttribLocation(shaderProgram, "position");
  glEnableVertexAttribArray(posAttrib);
  glVertexAttribPointer(posAttrib, 2, GL_FLOAT, GL_FALSE, 0, 0);

  emscripten_set_main_loop(update, 0, 1);

  return 0;
}
