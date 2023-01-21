#include <iostream>
#include "VisualizerOpenGL.h"
#include "Matrix.h"

VisualizerOpenGL* VisualizerOpenGL::CurrentInstance = NULL;
int main(int argc, char **argv) {
    //init some parameters
    double rev_mass = 0.1;
    double height = 4, radius = 2, h = 0.001, g = -9.81;
    Matrix rev_I(3, 3);
    rev_I(0, 0) = height * height / (12 * rev_mass) + radius * radius / (4 * rev_mass);
    rev_I(1, 1) = height * height / (12 * rev_mass) + radius * radius / (4 * rev_mass);
    rev_I(2, 2) = radius * radius / (2 * rev_mass);

    Params params = {rev_mass, height, radius, rev_I.Inv(), h, g};
    Matrix R(3, 3);
    R(0, 0) = 1;
    R(1, 1) = 1;
    R(2, 2) = 1;
    Matrix c(3, 1);
    c(2, 0) = height / 2 + 10;
    Matrix p(3, 1);
    Matrix L(3, 1);
    L(0, 0) = 0.1;

    State CenterState(c, p, R, L);
    auto Visual = new VisualizerOpenGL(CenterState, params);
    Visual->Init(argc, argv);
    return 0;
}
