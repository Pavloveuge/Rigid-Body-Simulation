#include "Calculator.h"

std::vector<double> Calculator::VectorProduct(std::vector<double> a, std::vector<double> b){
    std::vector<double> res = {0, 0, 0};
    res[0] = a[1] * b[2] - a[2] * b[1];
    res[1] = a[2] * b[0] - a[0] * b[2];
    res[2] = a[0] * b[1] - a[1] * b[0];
    return res;
}

double Calculator::ScalarProduct(std::vector<double> a, std::vector<double> b){
    double res = 0;
    for (int i = 0; i < 3; i++){
        res += a[i] * b[i];
    }
    return res;
}

std::vector<std::vector<double>> Calculator::T(std::vector<std::vector<double>> matrix){
    std::vector<std::vector<double>> res = {{0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            res[j][i] = matrix[i][j];
    return res;
}


std::vector<double> Calculator::MatrixOnVector(std::vector<std::vector<double>> matrix, std::vector<double> vec){
    std::vector<double> res = {0, 0, 0};
    for (int i = 0; i < 3; i++){
        res[i] = matrix[i][0] * vec[0] + matrix[i][1] * vec[1] + matrix[i][2] * vec[2];
    }
    return res;
}

std::vector<std::vector<double>> Calculator::Inv(std::vector<std::vector<double>> mat){
    double det = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) -
                 mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) +
                 mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);

    double invdet = 1 / det;

    std::vector<std::vector<double>> minv = {
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0}
    };
    minv[0][0] = (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) * invdet;
    minv[0][1] = (mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2]) * invdet;
    minv[0][2] = (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) * invdet;
    minv[1][0] = (mat[1][2] * mat[2][0] - mat[1][0] * mat[2][2]) * invdet;
    minv[1][1] = (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) * invdet;
    minv[1][2] = (mat[1][0] * mat[0][2] - mat[0][0] * mat[1][2]) * invdet;
    minv[2][0] = (mat[1][0] * mat[2][1] - mat[2][0] * mat[1][1]) * invdet;
    minv[2][1] = (mat[2][0] * mat[0][1] - mat[0][0] * mat[2][1]) * invdet;
    minv[2][2] = (mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1]) * invdet;

    return minv;
}


std::vector<double> Calculator::SumVector(std::vector<double> vec1, std::vector<double> vec2){
    std::vector<double> res = {vec1[0] + vec2[0],
                               vec1[1] + vec2[1],
                               vec1[2] + vec2[2]};
    return res;
}


std::vector<std::vector<double>> Calculator::MultMatrix(std::vector<std::vector<double>> matrix1, std::vector<std::vector<double>> matrix2){
    std::vector<std::vector<double>> res = {{0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}};
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            for (int k = 0; k < 3; k++){
                res[i][j] += matrix1[i][k] * matrix2[k][j];
            }
        }
    }

    return res;
}

double Calculator::Len(std::vector<double> a){
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

std::vector<std::vector<double>> Calculator::Ort(std::vector<std::vector<double>> mat){
    std::vector<double> f1 = { mat[0][0], mat[1][0] , mat[2][0] };
    std::vector<double> f2 = { mat[0][1], mat[1][1] , mat[2][1] };
    std::vector<double> f3 = { mat[0][2], mat[1][2] , mat[2][2] };
    std::vector<double> e1(3, 0), e2(3, 0), e3(3, 0);
    double g12, g13, g23;
    e1 = f1;
    g12 = ScalarProduct(f2, e1) / ScalarProduct(e1, e1);
    for (int i = 0; i < 3; i++){
        e2[i] = f2[i] - e1[i] * g12;
    }
    g13 = ScalarProduct(f3, e1) / ScalarProduct(e1, e1);
    g23 = ScalarProduct(f3, e2) / ScalarProduct(e2, e2);
    std::vector<double> p1 = {e1[0] * g13, e1[1] * g13, e1[2] * g13}, p2 = {e2[0] * g23, e2[1] * g23, e2[2] * g23};
    for (int i = 0; i < 3; i++){
        e3[i] = f3[i] - p1[i] -p2[i];
    }
    double a = std::sqrt(ScalarProduct(e1, e1));
    double b = std::sqrt(ScalarProduct(e2, e2));
    double c = std::sqrt(ScalarProduct(e2, e2));
    std::vector<std::vector<double>> res = {{e1[0] / a, e2[0] / b, e3[0] / c},
                                            {e1[1] / a, e2[1] / b, e3[1] / c},
                                            {e1[2] / a, e2[2] / b, e3[2] / c}};
    return res;

}