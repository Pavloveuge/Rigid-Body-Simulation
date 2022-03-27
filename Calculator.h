#ifndef RIGID_BODY_SIMULATION_CALCULATOR_H
#define RIGID_BODY_SIMULATION_CALCULATOR_H
#include <vector>
#include <cmath>

class Calculator {
    /*
     This class implement some operations from linear algebra
     */
public:
    std::vector<double> VectorProduct(std::vector<double> a, std::vector<double> b);
    double ScalarProduct(std::vector<double> a, std::vector<double> b);
    double Len(std::vector<double> a);
    std::vector<std::vector<double>> T(std::vector<std::vector<double>> matrix);
    std::vector<double> MatrixOnVector(std::vector<std::vector<double>> matrix, std::vector<double> vec);
    std::vector<double> SumVector(std::vector<double> vec1, std::vector<double> vec2);
    std::vector<std::vector<double>> MultMatrix(std::vector<std::vector<double>> matrix1, std::vector<std::vector<double>> matrix2);
    std::vector<std::vector<double>> Ort(std::vector<std::vector<double>> mat);
    std::vector<std::vector<double>> Inv(std::vector<std::vector<double>> mat);
};


#endif //RIGID_BODY_SIMULATION_CALCULATOR_H
