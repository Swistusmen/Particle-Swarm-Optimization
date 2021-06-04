#include "POSFunctions.h"

Positions::Positions(std::array<double, 2> x, std::array<double, 2> y, std::function<double(double, double)> fun, std::array<double,2> start)
{
	X = x;
	Y = y;
	goalFunction = fun;
	globalPosition = start;
	bestLocalPosition = start;
	currentPosition = start;
}

SwarmOutputata FindMinimum(SwarmInputData input)
{
	Positions* positions= new Positions[input.noParticles];
	std::vector<std::array<double, 2>> minimums(input.noParticles);
	std::vector<double> real_solutions(input.noParticles);
	
	for (int i = 0; i < input.noParticles; i++)
	{
		positions[i]= Positions{ input.X,input.Y,input.goalFunction,{GenerateStartingPosition(input.X),GenerateStartingPosition(input.Y)} };
	}

	for (int i = 0; i < input.noParticles; i++)
	{
		minimums[i] = positions[i].globalPosition;
		real_solutions[i] = input.goalFunction(minimums[i][0], minimums[i][1]);
	}
	auto bestSolution = minimums[std::min_element(real_solutions.begin(), real_solutions.end()) - real_solutions.begin()];

	for (int i = 0; i< input.noParticles; i++)
	{
		positions[i].globalPosition=bestSolutins
	}

}

SwarmOutputata SwarmOneThread(SwarmInputData input)
{
	std::vector<std::unique_ptr<Particle>> particles;
	std::vector<std::pair<double, double>> solutions;
	std::vector<double> real_solutions;
	const int noParticles = input.noParticles;
	const int n = input.iterations;
	for (int i = 0; i < noParticles; i++)
	{
		particles.push_back(std::make_unique<Particle>(input.iterations, input.goalFunction, input.X, input.Y));
		solutions.push_back(particles.back()->GetBestPosition());
		real_solutions.push_back(input.goalFunction(solutions.back().first, solutions.back().second));
	}

	auto bestSolution = solutions[std::min_element(real_solutions.begin(), real_solutions.end()) - real_solutions.begin()];

	for (int j = 0; j < noParticles; j++)
	{
		particles[j]->SetBestGlobalPosition(bestSolution);
	}


	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < noParticles; j++)
		{
			particles[j]->CalculateNextPosition();
			solutions[j] = particles[j]->GetBestPosition();
			real_solutions[j] = input.goalFunction(solutions[j].first, solutions[j].second);
		}
		auto tempBestSolution = solutions[std::min_element(solutions.begin(), solutions.end()) - solutions.begin()];
		if (input.goalFunction(tempBestSolution.first, tempBestSolution.second) < input.goalFunction(bestSolution.first, bestSolution.second))
		{
			bestSolution = tempBestSolution;
			for (int j = 0; j < noParticles; j++)
			{
				particles[j]->SetBestGlobalPosition(bestSolution);
			}
		}
	}
	SwarmOutputata output;

	output.x = bestSolution.first;
	output.y = bestSolution.second;
	output.z = input.goalFunction(output.x, output.y);

	return output;
}

int RandomInThousand()
{
	std::random_device rd;
	std::mt19937 gen{ rd() };
	std::normal_distribution<> dist(1000, 500);
	int scope = -1;
	while (scope < 0 && scope>1000)
		scope = dist(rd);
	return scope;
}

double GenerateStartingPosition(std::array<double, 2> min_max)
{
	double scope = std::abs(min_max[0] - min_max[1]);
	int number = -1;
	std::random_device rd;
	std::mt19937 gen{ rd() };
	std::normal_distribution<> dist(15000, 10000);
	while (number<0 || number>scope)
	{
		number = dist(rd);
	}
	return number + min_max[1];
}

Parameters CalculateParameters(int iteration, int noIterations)
{
	Parameters output;
	const double cmax = 2.5, cmin = 0.5; //according to to Ratnaweera
	double wmax = 0.9, wmin = 0.4; //accoring to menitoned method in w
	output.w= wmax - ((wmax - wmin) / noIterations) * iteration;
	output.c1= (cmin - cmax) * iteration / noIterations + cmax;
	output.c2= (cmax - cmin) * iteration / noIterations + cmin;
	return output;
}

std::array<double,2> FindBestDirection(Positions* position)
{
	std::vector<double> solutions(4);
	std::vector<std::array<double, 2>> directions(4);
	std::fill(solutions.begin(), solutions.end(), std::numeric_limits<double>::max());
	double velocity = position->velocity;
	double newPositiveX = position->bestLocalPosition[0] + velocity;
	double newNegativeX = position->bestLocalPosition[0] - velocity;
	double newPositiveY = position->bestLocalPosition[1] + velocity;
	double newNegativeY = position->bestLocalPosition[1] - velocity;
	double Xmax = position->X[0];
	double Xmin = position->X[1];
	double Ymax = position->Y[0];
	double Ymin = position->Y[1];
	
	if (newPositiveX < Xmax && newPositiveX > Xmin && newPositiveY < Ymax && newPositiveY > Ymin)
	{
		solutions[0] = position->goalFunction(newPositiveX, newPositiveY);
		directions[0] = { newPositiveX,newPositiveY };
	}
	if (newPositiveX < Xmax && newPositiveX > Xmin && newNegativeY < Ymax && newNegativeY > Ymin)
	{
		solutions[1] = position->goalFunction(newPositiveX, newNegativeY);
		directions[1] = { newPositiveX,newNegativeY };
	}
	if (newNegativeX < Xmax && newNegativeX > Xmin && newNegativeY < Ymax && newNegativeY > Ymin)
	{
		solutions[2] = position->goalFunction(newNegativeX, newNegativeY);
		directions[2] = { newNegativeX,newNegativeY };
	}
	if (newNegativeX < Xmax && newNegativeX > Xmin && newPositiveY < Ymax && newPositiveY > Ymin)
	{
		solutions[3] = position->goalFunction(newNegativeX, newPositiveY);
		directions[3] = { newNegativeX,newPositiveY };
	}
	int index = std::min_element(solutions.begin(), solutions.end()) - solutions.begin();
	return directions[index];
}

void CalculateBestLocalPosition(Positions* positions,Parameters parameters)
{
	std::array<double, 2> currentPosition = positions->currentPosition;
	double localDistance = std::sqrt(std::pow(positions->bestLocalPosition[0] - currentPosition[0], 2) + std::pow(positions->bestLocalPosition[0] - currentPosition[0], 2));
	double globalDistance = std::sqrt(std::pow(positions->globalPosition[0] - currentPosition[0], 2) + std::pow(positions->globalPosition[1] - currentPosition[1], 2));
	positions->velocity = positions->velocity * parameters.w + RandomInThousand()/1000.0 * parameters.c1 * localDistance + RandomInThousand() / 1000.0 * parameters.c2 * globalDistance;
	currentPosition = FindBestDirection(positions);
	positions->currentPosition = currentPosition;
	double oldPositionGoalFunctionValue = positions->goalFunction(positions->bestLocalPosition[0], positions->bestLocalPosition[1]);
	double newPositionGoalFunctionValue = positions->goalFunction(currentPosition[0], currentPosition[1]);
	if (newPositionGoalFunctionValue < oldPositionGoalFunctionValue)
		positions->bestLocalPosition = currentPosition;
}
