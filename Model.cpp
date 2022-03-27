#include "Model.h"

Model::Model(){

}

Model::Model(State* StartState, Params params) {
    this->CurrentState = StartState;
    this->params = params;
}

void Model::CheckCollisionCylinder(State *X, double radius, double height) {
    Calculator A;
    std::vector<double> top_corr = {0, 0, 0};
    std::vector<double> bot_corr = {0, 0, 0};
    std::vector<double> top_point = {0, 0, 0};
    std::vector<double> bot_point = {0, 0, 0};
    std::vector<double> par_imp = {0, 0, 0};
    std::vector<double> mid = {0, 0, 0};
    std::vector<double> mid_local = {0, 0, 0};
    std::vector<double> normal = {0, 0, 1};
    double pt_vel_top = 0, pt_vel_bot = 0, v_rel = 0;
    int cnt = 0;
    for (int k = 0; k < 3; k++){
        par_imp[k] = X->p[k] * this->params.rev_mass;
    }
    std::vector<std::vector<double>> Iinv = A.MultMatrix(X->R, A.MultMatrix(this->params.rev_I, A.T(X->R)));
    std::vector<double> omega = A.MatrixOnVector(Iinv, X->L);
    for (float i = 0.0; i <= 2 * PI; i += 0.05){
        top_corr = {cos(i)*radius, sin(i)*radius, height / 2};
        bot_corr = {cos(i)*radius, sin(i)*radius, -height / 2};
        top_point = A.SumVector(X->c, A.MatrixOnVector(X->R, top_corr));
        bot_point = A.SumVector(X->c, A.MatrixOnVector(X->R, bot_corr));
        if (top_point[2] < 0.01){
            pt_vel_top = A.SumVector(A.VectorProduct(omega, top_point), par_imp)[2];
            if (pt_vel_top < 0){
                mid = A.SumVector(mid, top_point);
                cnt++;
            }
        }
        if (bot_point[2] < 0){
            pt_vel_bot = A.SumVector(A.VectorProduct(omega, bot_point), par_imp)[2];
            if (pt_vel_bot < 0){
                mid = A.SumVector(mid, bot_point);
                cnt++;
            }
        }
    }
    for (int i = 0; i < 3; i++){
        mid[i] = mid[i] / cnt;
        mid_local[i] = mid[i] - X->c[i];
    }
    v_rel = A.SumVector(A.VectorProduct(omega, mid), par_imp)[2];
    double numerator = -2 * v_rel;
    double denominator = this->params.rev_mass + A.ScalarProduct(normal, A.VectorProduct(A.MatrixOnVector(Iinv, A.VectorProduct(mid_local, normal)), mid_local));
    double j = numerator / denominator;
    std::vector<double> force = {0, 0, j};
    std::vector<double> term = A.VectorProduct(mid_local, force);
    for (int i = 0 ; i < 3; i++){
        X->p[i] += force[i];
        X->L[i] += term[i];
    }
}

bool Model::CheckPoints(State *X, double radius, double height) {
    Calculator A;
    std::vector<double> top_corr = {0, 0, 0};
    std::vector<double> bot_corr = {0, 0, 0};
    std::vector<double> top_point = {0, 0, 0};
    std::vector<double> bot_point = {0, 0, 0};
    std::vector<double> par_imp = {0, 0, 0};
    double pt_vel_top = 0, pt_vel_bot = 0;
    for (int k = 0; k < 3; k++){
        par_imp[k] = X->p[k] * this->params.rev_mass;
    }
    std::vector<std::vector<double>> Iinv = A.MultMatrix(X->R, A.MultMatrix(this->params.rev_I, A.T(X->R)));
    std::vector<double> omega = A.MatrixOnVector(Iinv, X->L);
    for (float i = 0.0; i <= 2 * PI; i += 0.05){
        top_corr = {cos(i)*radius, sin(i)*radius, height / 2};
        bot_corr = {cos(i)*radius, sin(i)*radius, -height / 2};
        top_point = A.SumVector(X->c, A.MatrixOnVector(X->R, top_corr));
        bot_point = A.SumVector(X->c, A.MatrixOnVector(X->R, bot_corr));
        if (top_point[2] < 0.01){
            pt_vel_top = A.SumVector(A.VectorProduct(omega, top_point), par_imp)[2];
            if (pt_vel_top < 0){
                return false;
            }
        }
        if (bot_point[2] < 0){
            pt_vel_bot = A.SumVector(A.VectorProduct(omega, bot_point), par_imp)[2];
            if (pt_vel_bot < 0){
                return false;
            }
        }
    }
    return true;
}

void Model::f(State *X, State *Xdot){
    for (int i=0; i < 3; i++){
        Xdot->c[i] = X->p[i] * this->params.rev_mass;
    }
    Xdot->p[2] = - 9.81 / this->params.rev_mass;
    Calculator A = Calculator();
    std::vector<std::vector<double>> inv = A.MultMatrix(A.MultMatrix(X->R, this->params.rev_I), A.T(X->R));
    std::vector<double> omega = A.MatrixOnVector(inv, X->L);
    std::vector<std::vector<double>> skew = {{0, -omega[2], omega[1]},
                                             {omega[2], 0, -omega[0]},
                                             {-omega[1], omega[0], 0}};
    Xdot->R = A.MultMatrix(skew, X->R);
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
    while (not(CheckPoints(CurrentState, params.radius, params.height))){
        CheckCollisionCylinder(CurrentState, params.radius, params.height);
    }
    //CurrentState->L = CurrentState->L * this->params[1];
    //CurrentState->p = CurrentState->p * this->params[1];
}
