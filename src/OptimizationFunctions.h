#include "Particle.h"
#include <memory>
#include <utility>

struct SwarmInputData {
	std::function<double(double, double)> goalFunction;
	std::array<double, 2> X;
	std::array<double, 2> Y;
	int iterations;
	int noParticles;
	double mean, srandardDeviation;
};

struct SwarmOutputata {
	double x, y, z;
};

SwarmOutputata SwarmOneThread(SwarmInputData data);