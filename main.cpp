#include <iostream>
#include "VisualizerOpenGL.h"
#include "Matrix.h"

VisualizerOpenGL* VisualizerOpenGL::CurrentInstance = NULL;
int main(int argc, char **argv) {
    //init some parameters
    double rev_mass = 0.1;
    double size_size = 4, h = 0.01, g = -9.81;
    Matrix rev_I(3, 3);
    rev_I(0, 0) = 2 / (12 * rev_mass);
    rev_I(1, 1) = 2 / (12 * rev_mass);
    rev_I(2, 2) = 2 / (12 * rev_mass);

    Params params = {rev_mass, size_size, rev_I.Inv(), h, g};
    Matrix R(3, 3);
    R(0, 0) = 1;
    R(1, 1) = 1;
    R(2, 2) = 1;
    Matrix c(3, 1);
    c(2, 0) = size_size * 1.5;
    Matrix p(3, 1);
    Matrix L(3, 1);
    L(0, 0) = 0.6;

    State CenterState(c, p, R, L);
    auto Visual = new VisualizerOpenGL(CenterState, params);
    Visual->Init(argc, argv);
    return 0;
}
