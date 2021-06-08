#include "OptimizationFunctions.h"
#include "SavePoints.h"
#include "POSFunctions.h"
#include <iostream>
#include <chrono>

int main(){
    std::array<double, 2> x;
    std::array<double, 2> y;
    x[1] = -5.0;
    x[0] = 5.0;
    y[1] = -5.0;
    y[0] = 5.0;
    auto Himmelblau = [](double x, double y) {return std::pow(x * x + y - 11, 2) + std::pow(x + y * y - 7, 2); };
    SwarmInputData input;
    input.X = x;
    input.Y = y;
    input.noParticles = 100;
    input.iterations = 1000;
    input.goalFunction = Himmelblau;
    input.threads = 4;
    auto start = std::chrono::system_clock::now();
    auto output = SwarmOneThread(input);
    auto end = std::chrono::system_clock::now();
    std::cout << (end - start).count() << std::endl;
    std::cout << "x: " << output.x << " y: " << output.y << " z: " << output.z << std::endl;
    SavePoints(output.minimums, Himmelblau, "data1.txt");
    
    start = std::chrono::system_clock::now();
    output = FindMinimum(input);
    end = std::chrono::system_clock::now();
    std::cout << (end - start).count() << std::endl;
    std::cout << "x: " << output.x << " y: " << output.y << " z: " << output.z << std::endl;
    std::cout << output.minimums.size() << std::endl;
    
    SavePoints(output.minimums, Himmelblau, "data2.txt");

    return 0;
}