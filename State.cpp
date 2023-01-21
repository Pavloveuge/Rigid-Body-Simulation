#include "State.h"

State::State(){
    this->c = Matrix(3, 1);
    this->R = Matrix(3, 3);
    this->p = Matrix(3, 1);
    this->L = Matrix(3, 1);
}

State::State(Matrix c, Matrix p,
             Matrix R, Matrix L){
    this->c = c;
    this->R = R;
    this->p = p;
    this->L = L;
}


State operator+(const State &d1, const State &d2)
{
    State res;
    for (int i = 0; i < 3; i++){
        res.c(i, 0) = d1.c(i, 0) + d2.c(i, 0);
        res.p(i, 0) = d1.p(i, 0) + d2.p(i, 0);
        res.L(i, 0) = d1.L(i, 0) + d2.L(i, 0);
        for (int j = 0; j < 3; j++){
            res.R(i, j) = d1.R(i, j) + d2.R(i, j);
        }
    }
    return res;
}

State operator*(const double &d1, const State &d2){
    State res;
    for (int i = 0; i < 3; i++){
        res.c(i, 0) = d1 * d2.c(i, 0);
        res.p(i, 0) = d1 * d2.p(i, 0);
        res.L(i, 0) = d1 * d2.L(i, 0);
        for (int j = 0; j < 3; j++){
            res.R(i, j) = d1 * d2.R(i, j);
        }
    }
    return res;
}
