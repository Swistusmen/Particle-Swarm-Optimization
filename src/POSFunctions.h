#pragma once
#include "Utilities.h"
#include <random>
#include <thread>
#include <barrier>
#include <memory>
#include <utility>

struct Parameters {
	double c1=0.0, c2=0.0, w=0.0;
	Parameters() = default;
};

struct Positions {
	Positions() = default;
	Positions(std::array<double, 2> x, std::array<double, 2> y, std::function<double(double, double)> fun, std::array<double, 2> start);
	std::array<double, 2> bestLocalPosition;
	std::array<double, 2> globalPosition;
	std::array<double, 2> currentPosition;
	std::array<double, 2> X;
	std::array<double, 2> Y;
	std::function<double(double, double)> goalFunction;
	double velocity = 0.0;
};

int RandomInThousand(); //for r and starting velocity
double GenerateStartingPosition(std::array<double,2> min_max);
Parameters CalculateParameters(int iteration, int noIterations);

void CalculateBestLocalPosition(Positions* positions, Parameters parameters);

std::array<double, 2> FindBestDirection(Positions* positions);

std::vector<std::array<int, 2>> CalculateThreadBounds(SwarmInputData* input);

SwarmOutputata FindMinimum(SwarmInputData input);

//void CalculateNextMove(std::array<int, 2> range, Positions* positions, Parameters* params, int iteration,int noIterations);

void CalculateNextMove(std::array<int, 2> range, Positions* positions, Parameters* params,
	int noIterations,std::array<double,2>& bestSolution,std::vector<double>& real_solutions,
	std::vector<std::array<double,2>>& minimums, std::function<double(double, double)> fun, std::barrier& bar);
