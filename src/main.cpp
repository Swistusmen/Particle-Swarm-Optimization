#include "OptimizationFunctions.h"
#include "SavePoints.h"
#include "POSFunctions.h"
#include <iostream>
#include <chrono>

#define M_PI 3.14159

int main(){
    std::array<double, 2> x;
    std::array<double, 2> y;
    x[1] = -5.0;
    x[0] = 5.0;
    y[1] = -5.0;
    y[0] = 5.0;
    auto Himmelblau = [](double x, double y) {return std::pow(x * x + y - 11, 2) + std::pow(x + y * y - 7, 2); };
    
    auto Levi = [](double x, double y) {
        return std::pow(std::sin(3 * M_PI * x), 2) + std::pow(x - 1, 2) * (1 + std::pow(std::sin(3 * y * M_PI), 2))
            + std::pow(y - 1, 2) * (1 + std::pow(std::sin(2 * M_PI * y), 2));
    };
    auto Rastrign = [](double x, double y) {
        return
            20 + std::pow( x, 2) - 10 * std::cos(2 * M_PI * x) + y * y - 10 * std::cos(2 * M_PI * y);
    };
    SwarmInputData input;
    input.X = x;
    input.Y = y;
    input.noParticles = 100;
    input.iterations = 1000;
    input.goalFunction = Levi;
    input.threads = 4;
    auto start = std::chrono::system_clock::now();
    auto output = SwarmOneThread(input);
    auto end = std::chrono::system_clock::now();
    std::cout << (end - start).count() <<" "<<std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << std::endl;
    std::cout << "x: " << output.x << " y: " << output.y << " z: " << output.z << std::endl;
    SavePoints(output.minimums, input.goalFunction, "data1.txt");
    
    start = std::chrono::system_clock::now();
    output = FindMinimum(input);
    end = std::chrono::system_clock::now();
    std::cout << (end - start).count() << std::endl;
    std::cout << "x: " << output.x << " y: " << output.y << " z: " << output.z << std::endl;
    
    SavePoints(output.minimums, input.goalFunction, "data2.txt");

    return 0;
}