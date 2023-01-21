#ifndef PARAVM_VISUALIZEROPENGL_H
#define PARAVM_VISUALIZEROPENGL_H
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include "Model.h"
#include "Matrix.h"


class VisualizerOpenGL{
    /*
     This class using for visualisation. I used Callback to work with OpenGL in OOP style
     */
public:
    static void RenderCallback();
    static void ChangeCallback(int w, int h);
    static void TimerCallback(int value);
    void ShowFloor();
    void DrawCube();
    static VisualizerOpenGL* CurrentInstance;
    VisualizerOpenGL();
    VisualizerOpenGL(State& state, Params params);
    void ChangeSize(int w, int h);
    void RenderScene();
    void Init(int argc, char **argv);
private:
    Model m;
};


#endif //PARAVM_VISUALIZEROPENGL_H