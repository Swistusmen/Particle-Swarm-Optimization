#pragma once
#include "Utilities.h"

struct Parameters {
	double c1, c2, w;
};

struct Positions {
	std::array<double, 2> bestLocalPosition;
	std::array<double, 2> globalPosition;
};

double RandomInRange(); //for r and starting velocity
double GenerateStartingPosition(std::array<double,2> min_max);
Parameters CalculateParameters(int iteration, int noIterations);

void CalculateBestLocalPosition(Positions* positions);

double FindBestDirection(std::array<double, 2> position, double velocity);

