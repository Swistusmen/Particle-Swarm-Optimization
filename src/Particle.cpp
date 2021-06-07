#include "Particle.h"

Particle::Particle(int numberOfIterations, std::function< double( double,  double)> fun, std::array< double,2> maxX, std::array< double,2> maxY)
{
	goalFunction = fun;
	T = numberOfIterations;
	X = maxX;
	Y = maxY;
	currentPosition = GenerateStartingPositions(maxX, maxY);
	bestLocalPosition = currentPosition;
	bestPosition = currentPosition;
	std::normal_distribution<> dist(1000, 500);
	int scope = -1;
	while (scope < 0 && scope>1000)
		scope = dist(rd);
	
	velocity = scope / 1000;
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
	n++;
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
	double scopeX = std::abs(x[0] - x[1]);
	double scopeY = std::abs(y[0]-y[1]);

	int X = scopeX * EPSILON_EXP;
	int Y = scopeY * EPSILON_EXP;

	std::pair< double, double> result;

	std::normal_distribution<> dist(15000 , 10000);
	srand(std::time(NULL));
	while (true)
	{
		result.first = std::abs(dist(rd));
		result.first /= EPSILON_EXP;
		result.first += x[1];
		if (result.first <= x[0] && result.first > x[1])
			break;
	}
	
	while (true)
	{
		result.second = std::abs(dist(rd));
		result.second /= EPSILON_EXP;
		result.second += y[1];
		if (result.second <= y[0] && result.second > y[1])
			break;
	}

	return result;
}

void Particle::RandomizeR()
{
	std::normal_distribution<> dist(500 , 250 );
	while (true)
	{
		rl = std::abs(dist(rd));
		if (rl < 1000 && rl>0)
			break;
	}
	while (true)
	{
		rg = std::abs(dist(rd));
		if (rg < 1000 && rg>0)
			break;
	}
	rl /= 1000;
	rg /= 1000;
}

void Particle::SetBestGlobalPosition(std::pair<double, double> solution)
{
	double newSolution = goalFunction(solution.first, solution.second);
	double oldSolution = goalFunction(bestPosition.first, bestPosition.second);
	if (oldSolution > newSolution)
		bestPosition = solution;
}
