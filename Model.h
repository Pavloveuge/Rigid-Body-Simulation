#ifndef RIGID_BODY_SIMULATION_MODEL_H
#define RIGID_BODY_SIMULATION_MODEL_H
#include "State.h"
#include <iostream>
#define PI 3.14159265359

class Model {
    /*
     This class is physical model of falling cylinder.
     You can read README to look on the math model of process.
     For differential equations was used RK4 method.
     */
public:
    Model(State StartState, Params params);
    Model();
    void NextRK4(); // step of RK4
    void CheckCollisionCube(State* X);
    bool CheckPoints(State* X);
    State GetState();
    Params GetParams();
private:
    State* CurrentState;
    Params params;
    void f(State *X, State *Xdot); // calculate derivations
};


#endif //RIGID_BODY_SIMULATION_MODEL_H
