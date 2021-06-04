#pragma once
#include <array>
#include <functional>

struct SwarmInputData {
	std::function<double(double, double)> goalFunction;
	std::array<double, 2> X;
	std::array<double, 2> Y;
	int iterations;
	int noParticles;
};

struct SwarmOutputata {
	double x, y, z;
};