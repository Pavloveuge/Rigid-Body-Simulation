#ifndef RIGID_BODY_SIMULATION_STATE_H
#define RIGID_BODY_SIMULATION_STATE_H
#include <vector>
#include "Matrix.h"

struct State{
    //Struct, which describe material point
public:
    Matrix c;
    Matrix R;
    Matrix p;
    Matrix L;
    State();
    State(Matrix c, Matrix p,
          Matrix R, Matrix L);
    friend State operator+(const State &d1, const State &d2);
    friend State operator*(const double &d1, const State &d2);
};



struct Params{
    //Struct, which store some parameters
    double rev_mass; // inverse mass
    double height;
    double radius;
    Matrix rev_I; // inverse inertia tensor
    double h; // step of integrate
    double g; // gravity coefficient
};

#endif //RIGID_BODY_SIMULATION_STATE_H
