#ifndef RIGID_BODY_SIMULATION_STATE_H
#define RIGID_BODY_SIMULATION_STATE_H
#include <vector>

struct State{
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
    double rev_mass;
    double height;
    double radius;
    std::vector<std::vector<double>> rev_I;
    double h;
    double g;
};

#endif //RIGID_BODY_SIMULATION_STATE_H
