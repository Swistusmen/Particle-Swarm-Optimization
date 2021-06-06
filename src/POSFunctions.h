#pragma once
#include "Utilities.h"
#include <random>
#include <thread>
#include <barrier>
#include <memory>
#include <utility>
#include <mutex>
#include <iostream>

struct Parameters {
	double c1=0.0, c2=0.0, w=0.0;
	Parameters() = default;
};

struct ThreadCommon {
	std::thread::id sorterID;
	std::thread::id mainThread;
	bool readyToCalc = false;
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

struct NextMove {
	Positions* positions;
	Parameters* params;
	SwarmInputData input;
	std::array<double, 2>* bestSolution;
	std::vector<std::array<double, 2>>* minimums;
	std::vector<double>* real_solutions;
	std::array<int, 2> range;
	std::mutex* mymut;
	int* isJobDone;
	int number;
	ThreadCommon* tCommon;

	NextMove(Parameters* params, Positions* pos, SwarmInputData inp, std::array<double, 2>* sol,
		std::vector<std::array<double, 2>>* mins, std::vector<double>* rsol, std::array<int, 2> r,
		std::mutex* m,int* tabOfJobs,int number,ThreadCommon* tcom);
};

int RandomInThousand(); //for r and starting velocity
double GenerateStartingPosition(std::array<double,2> min_max);
Parameters CalculateParameters(int iteration, int noIterations);

void CalculateBestLocalPosition(Positions* positions, Parameters parameters);

std::array<double, 2> FindBestDirection(Positions* positions);

std::vector<std::array<int, 2>> CalculateThreadBounds(SwarmInputData* input);

SwarmOutputata FindMinimum(SwarmInputData input);

//void CalculateNextMove(std::array<int, 2> range, Positions* positions, Parameters* params, int iteration,int noIterations);

void CalculateNextMove(NextMove next);
