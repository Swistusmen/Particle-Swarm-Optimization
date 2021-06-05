#pragma once
#include <array>
#include <functional>

#define EPSILON 0.001
#define EPSILON_EXP 1000
#define VELOCITY_SCOPE 100
#define SIGMA 0.5

struct SwarmInputData {
	std::function<double(double, double)> goalFunction;
	std::array<double, 2> X;
	std::array<double, 2> Y;
	int iterations;
	int noParticles;
	int threads;
};

struct SwarmOutputata {
	double x, y, z;
};