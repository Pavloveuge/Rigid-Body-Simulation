#ifndef RIGID_BODY_SIMULATION_STATE_H
#define RIGID_BODY_SIMULATION_STATE_H
#include <vector>

struct State{
    //Struct, which describe material point
public:
    std::vector<double> c;
    std::vector<std::vector<double>> R;
    std::vector<double> p;
    std::vector<double> L;
    State();
    State(std::vector<double> c, std::vector<double> p,
          std::vector<std::vector<double>> R, std::vector<double> L);
    friend State operator+(const State &d1, const State &d2);
    friend State operator*(const double &d1, const State &d2);
};



struct Params{
    //Struct, which store some parameters
    double rev_mass; // inverse mass
    double height;
    double radius;
    std::vector<std::vector<double>> rev_I; // inverse inertia tensor
    double h; // step of integrate
    double g; // gravity coefficient
};

#endif //RIGID_BODY_SIMULATION_STATE_H
