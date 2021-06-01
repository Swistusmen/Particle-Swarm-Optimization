#include "OptimizationFunctions.h"
#include <iostream>

double a(double x, double y) { return x * y; }

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
    input.noParticles = 2;
    input.iterations = 10;
    input.goalFunction = Himmelblau;

    auto output = SwarmOneThread(input);
    std::cout << "x: " << output.x << " y: " << output.y << " z: " << output.z << std::endl;
	//add input
    //add setting up and checking best global solution
    //implement working flow
    //check on real function
    //add multithreading
    return 0;
}