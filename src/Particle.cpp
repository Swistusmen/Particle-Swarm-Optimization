#include "Particle.h"
#include <iostream>

Particle::Particle(int numberOfIterations, std::function< double( double,  double)> fun, std::array< double,2> maxX, std::array< double,2> maxY)
{
	goalFunction = fun;
	T = numberOfIterations;
	X = maxX;
	Y = maxY;
	currentPosition = GenerateStartingPositions(maxX, maxY);
	bestLocalPosition = currentPosition;
	bestPosition = currentPosition;
	srand(std::time(nullptr));
	velocity = (rand() % VELOCITY_SCOPE) / std::pow(10, rand() % VELOCITY_SCOPE + 1);
	RandomizeR();
}

void Particle::CalculateNextPosition()
{
	c1 = (cmin - cmax) * n / T + cmax;
	c2 = (cmax - cmin) * n / T + cmin;
	w = wmax - ((wmax - wmin) / T) * n;
	double localDistance = std::sqrt(std::pow(bestLocalPosition.first - currentPosition.first, 2) + std::pow(bestLocalPosition.second - currentPosition.second, 2));
	double globalDistance= std::sqrt(std::pow(bestPosition.first - currentPosition.first, 2) + std::pow(bestPosition.second - currentPosition.second, 2));
	velocity = velocity * w + rl * c1 * localDistance + rg * c2 * globalDistance;
	currentPosition = FindBestDirection(velocity);
	double oldPositionGoalFunctionValue = goalFunction(bestLocalPosition.first, bestLocalPosition.second);
	double newPositionGoalFunctionValue = goalFunction(currentPosition.first, currentPosition.second);
	if (newPositionGoalFunctionValue < oldPositionGoalFunctionValue)
		bestLocalPosition = currentPosition;
}

std::pair<double, double> Particle::FindBestDirection(double velocity)
{
	std::vector<double> solutions(4);
	std::vector<std::pair<double, double>> directions(4);
	std::fill(solutions.begin(), solutions.end(), std::numeric_limits<double>::max());
	double newPositiveX = currentPosition.first + velocity;
	double newNegativeX = currentPosition.first - velocity;
	double newPositiveY = currentPosition.second + velocity;
	double newNegativeY = currentPosition.second - velocity;
	if (newPositiveX < X[0] && newPositiveX > X[1] && newPositiveY < Y[0] && newPositiveY > Y[1])
	{
		solutions[0] = goalFunction(newPositiveX, newPositiveY);
		directions[0] = { newPositiveX,newPositiveY };
	}
	if (newPositiveX < X[0] && newPositiveX > X[1] && newNegativeY < Y[0] && newNegativeY > Y[1])
	{
		solutions[1] = goalFunction(newPositiveX, newNegativeY);
		directions[1] = { newPositiveX,newNegativeY };
	}
	if (newNegativeX < X[0] && newNegativeX > X[1] && newNegativeY < Y[0] && newNegativeY > Y[1])
	{
		solutions[2] = goalFunction(newNegativeX, newNegativeY);
		directions[2] = { newNegativeX,newNegativeY };
	}
	if (newNegativeX < X[0] && newNegativeX > X[1] && newPositiveY < Y[0] && newPositiveY > Y[1])
	{
		solutions[3] = goalFunction(newNegativeX, newPositiveY);
		directions[3] = { newNegativeX,newPositiveY };
	}
	int index = std::min_element(solutions.begin(), solutions.end()) - solutions.begin();
	return directions[index];
}

std::pair<double, double> Particle::GenerateStartingPositions(std::array< double, 2> x, std::array< double, 2> y)
{
	 double scopeX = std::abs(x[0]) - std::abs(x[1]);
	 double scopeY = std::abs(y[0]) - std::abs(y[1]);

	int X = scopeX * EPSILON_EXP;
	int Y = scopeY * EPSILON_EXP;

	std::pair< double,  double> result;
	srand(std::time(nullptr));
	result.first = (rand() % X);
	result.second = (rand() % Y);
	if (rand() % 1 == 0)
	{
		result.first = std::abs(x[0]) * EPSILON_EXP - result.first;
		result.second = std::abs(y[0]) * EPSILON_EXP - result.second;
	}
	result.first *= EPSILON;
	result.first += x[1];
	result.second *= EPSILON;
	result.second += y[1];
	return result;
}

void Particle::RandomizeR()
{
	srand(std::time(nullptr));
	int max = rand() % 33;
	for(int i=0;i<max;i++)
		rl = static_cast<double>((rand() % 1001) / 1000.0);
	max = rand() % 17;
	for (int i = 0; i < max; i++)
		rg = static_cast<double>((rand() % 1001) / 1000.0);
}
