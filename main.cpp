#include <iostream>
#include "VisualizerOpenGL.h"

VisualizerOpenGL* VisualizerOpenGL::CurrentInstance = NULL;
int main(int argc, char **argv) {
    double rev_mass = 0.0001;
    double height = 4, radius = 2, h = 0.001, g = -9.81;
    std::vector<std::vector<double>> rev_I = {
            {height * height / (12 * rev_mass) + radius * radius / (4 * rev_mass), 0, 0},
            {0, height * height / (12 * rev_mass) + radius * radius / (4 * rev_mass), 0},
            {0, 0, radius * radius / (2 * rev_mass)}
    };
    Calculator B;
    Params params = {rev_mass, height, radius, B.Inv(rev_I), h, g};
    std::vector<std::vector<double>> R = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
    };
    std::vector<double> c = {0, 0, height / 2 + 10};
    std::vector<double> p = {0, 0, 0};
    std::vector<double> L = {0.1, 0, 0};

    State CenterState(c, p, R, L);
    auto Visual = new VisualizerOpenGL(CenterState, params);
    Visual->Init(argc, argv);
    return 0;
}
