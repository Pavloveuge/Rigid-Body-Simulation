#include "Model.h"

Model::Model(){

}

Model::Model(State StartState, Params params) {
    this->CurrentState = new State;
    this->CurrentState->c = StartState.c;
    this->CurrentState->R = StartState.R;
    this->CurrentState->p = StartState.p;
    this->CurrentState->L = StartState.L;
    this->params = params;
}

void Model::CheckCollisionCylinder(State *X, double radius, double height) {
    /*
     This method check that all points are above the floor and implements logic of collision if not
     */
    Matrix top_corr(3, 1);
    Matrix bot_corr(3, 1);
    Matrix top_point(3, 1);
    Matrix bot_point(3, 1);
    Matrix par_imp(3, 1);
    Matrix mid(3, 1);
    Matrix mid_local(3, 1);
    Matrix normal(3, 1);
    normal(2, 0) = 1;
    double pt_vel_top = 0, pt_vel_bot = 0, v_rel = 0;
    int cnt = 0;
    for (int k = 0; k < 3; k++){
        par_imp(k, 0) = X->p(k, 0) * this->params.rev_mass;
    }
    Matrix Iinv = X->R * this->params.rev_I * (X->R).T();
    Matrix omega = Iinv * X->L;
    for (float i = 0.0; i <= 2 * PI; i += 0.05){
        top_corr(0, 0) =  cos(i)*radius; top_corr(1, 0) = sin(i) * radius; top_corr(2, 0) = height / 2;
        bot_corr(0, 0) =  cos(i)*radius; bot_corr(1, 0) = sin(i) * radius; bot_corr(2, 0) = -height / 2;
        top_point = X->c + X->R * top_corr;
        bot_point = X->c + X->R * bot_corr;

        if (top_point(2, 0) < 0.01){
            pt_vel_top = (Matrix::VectorProduct(omega, top_point) + par_imp)(2, 0);
            if (pt_vel_top < 0){
                mid = mid + top_point;
                cnt++;
            }
        }

        if (bot_point(2, 0) < 0){
            pt_vel_bot = (Matrix::VectorProduct(omega, bot_point) + par_imp)(2, 0);
            if (pt_vel_bot < 0){
                mid = mid + bot_point;
                cnt++;
            }
        }
    }


    for (int i = 0; i < 3; i++){
        mid(i, 0) = mid(i, 0) / cnt;
        mid_local(i, 0) = mid(i, 0) - X->c(i, 0);
    }

    Matrix s = Matrix::VectorProduct(omega, mid) + par_imp;
    v_rel = s(2, 0);
    double numerator = -2 * v_rel;
    Matrix Some = normal * Matrix::VectorProduct((Iinv * Matrix::VectorProduct(mid_local, normal)), mid_local).T();
    double denominator = this->params.rev_mass + Some(0,0);//+ normal * Matrix::VectorProduct(Iinv * Matrix::VectorProduct(mid_local, normal), mid_local);
    double j = numerator / denominator;
    Matrix force(3, 1);
    force(2, 0) =  j;
    Matrix term = Matrix::VectorProduct(mid_local, force);
    for (int i = 0 ; i < 3; i++){
        X->p(i, 0) += force(i, 0);
        X->L(i, 0) += term(i, 0);
    }
}

bool Model::CheckPoints(State *X, double radius, double height) {
    /*
     This method using to check that all points are above the floor
     Because this model of cylinder enough to check extreme points(top and bottom of figure)
     */
    Matrix top_corr(3, 1);
    Matrix bot_corr(3, 1);
    Matrix top_point(3, 1);
    Matrix bot_point(3, 1);
    Matrix par_imp(3, 1);
    double pt_vel_top = 0, pt_vel_bot = 0;

    for (int k = 0; k < 3; k++){
        par_imp(k, 0) = X->p(k, 0) * this->params.rev_mass;
    }

    Matrix Iinv = X->R * this->params.rev_I * (X->R).T();
    Matrix omega = Iinv * X->L;
    for (float i = 0.0; i <= 2 * PI; i += 0.05){
        top_corr(0, 0) =  cos(i)*radius; top_corr(1, 0) = sin(i) * radius; top_corr(2, 0) = height / 2;
        bot_corr(0, 0) =  cos(i)*radius; bot_corr(1, 0) = sin(i) * radius; bot_corr(2, 0) = -height / 2;
        top_point = X->c + X->R * top_corr;
        bot_point = X->c + X->R * bot_corr;

        if (top_point(2, 0) < 0.01){ // check top
            pt_vel_top = (Matrix::VectorProduct(omega, top_point) + par_imp)(2, 0);
            if (pt_vel_top < 0){
                std::cout << "top";
                return false;
            }
        }

        if (bot_point(2, 0) < 0){ // check bottom
            pt_vel_bot = (Matrix::VectorProduct(omega, bot_point) + par_imp)(2, 0);
            if (pt_vel_bot < 0){
                std::cout << "bot";
                return false;
            }
        }
    }
    return true;
}

void Model::f(State *X, State *Xdot){

    for (int i=0; i < 3; i++){
        Xdot->c(i, 0) = X->p(i, 0) * this->params.rev_mass; // derivative of coordinate
    }
    Xdot->p(2, 0) = params.g / this->params.rev_mass; // derivative of momentum
    Matrix inv = X->R * this->params.rev_I * (X->R).T();
    Matrix omega = inv * X->L;
    Matrix skew(3, 3);
    skew(0, 1) = -omega(2, 0); skew(0, 2) = omega(1, 0);
    skew(1, 0) = omega(2, 0); skew(1, 2) = -omega(0, 0);
    skew(2, 0) = -omega(1, 0); skew(2, 1) = omega(0, 0);
    Xdot->R = skew * X->R; // derivative of rotate matrix
}

State Model::GetState(){
    return *(this->CurrentState);
}

Params Model::GetParams(){
    return this->params;
}

void Model::NextRK4(){
    State Xdot;
    State k1, k2, k3, k4, prom;
    f(CurrentState, &k1);
    prom = *CurrentState + (params.h / 2) * k1;
    f(&prom, &k2);
    prom = *CurrentState + (params.h / 2) * k2;
    f(&prom, &k3);
    prom = *CurrentState + params.h * k3;
    f(&prom, &k4);
    Xdot = (k1 + 2 * k2 + 2 * k3 + k4);
    *CurrentState = *CurrentState + params.h / 6 * Xdot;
    std::cout << "some";
    while (not(CheckPoints(CurrentState, params.radius, params.height))){
        CheckCollisionCylinder(CurrentState, params.radius, params.height);
    }
}
