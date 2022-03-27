#ifndef RIGID_BODY_SIMULATION_MODEL_H
#define RIGID_BODY_SIMULATION_MODEL_H
#include "State.h"
#include "Calculator.h"
#define PI 3.14159265359

class Model {
public:
    Model(State* StartState, Params params);
    Model();
    void NextRK4();
    void CheckCollisionCylinder(State* X, double radius, double height);
    bool CheckPoints(State* X, double radius, double height);
    State GetState();
    Params GetParams();
private:
    State* CurrentState;
    Params params;
    void f(State *X, State *Xdot);
};


#endif //RIGID_BODY_SIMULATION_MODEL_H
