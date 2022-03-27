#include "State.h"

State::State(){
    this->c = {0, 0, 0};
    this->R = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
    };
    this->p = {0, 0, 0};
    this->L = {0, 0, 0};
}

State::State(std::vector<double> c, std::vector<double> p,
             std::vector<std::vector<double>> R, std::vector<double> L){
    this->c = c;
    this->R = R;
    this->p = p;
    this->L = L;
}


State operator+(const State &d1, const State &d2)
{
    State res;
    for (int i = 0; i < 3; i++){
        res.c[i] = d1.c[i] + d2.c[i];
        res.p[i] = d1.p[i] + d2.p[i];
        res.L[i] = d1.L[i] + d2.L[i];
        for (int j = 0; j < 3; j++){
            res.R[i][j] = d1.R[i][j] + d2.R[i][j];
        }
    }
    return res;
}

State operator*(const double &d1, const State &d2){
    State res;
    for (int i = 0; i < 3; i++){
        res.c[i] = d1 * d2.c[i];
        res.p[i] = d1 * d2.p[i];
        res.L[i] = d1 * d2.L[i];
        for (int j = 0; j < 3; j++){
            res.R[i][j] = d1 * d2.R[i][j];
        }
    }
    return res;
}
