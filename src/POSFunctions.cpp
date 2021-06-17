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

SwarmOutputata FindMinimumAsync(SwarmInputData input)
{
	std::vector<Positions> positions(input.noParticles);
	std::generate(positions.begin(), positions.end(), [&]() {
		return Positions{ input.X,input.Y,input.goalFunction,
		{GenerateStartingPosition(input.X),GenerateStartingPosition(input.Y)} }; }
	);
	std::vector<std::array<double, 2>> minimums(input.noParticles);
	std::vector<double> solutions(input.noParticles);
	const std::function<double(double, double)> fun = input.goalFunction;
	std::transform(positions.begin(), positions.end(), minimums.begin(), [](Positions pos) {return pos.globalPosition; });
	std::transform(solutions.begin(), solutions.end(), minimums.begin(), [fun](std::array<double, 2> val) {
		return fun(val[0], val[1]); });
	auto bestSolution = minimums[std::min_element(solutions.begin(), solutions.end()) - solutions.begin()];
	std::for_each(positions.begin(), positions.end(), [bestSolution](Positions& pos) {
		return pos.globalPosition = bestSolution;
		});
	const int max = input.iterations;
	for (int i = 0; i < max; i++)
	{
		//funkcja ktora zwraca wektor pozycji
	}
	SwarmOutputata out;
	return out;
}

SwarmOutputata FindMinimum(SwarmInputData input)
{
	auto threadRanges = CalculateThreadBounds(&input);
	Positions* positions= new Positions[input.noParticles];

	std::vector<std::array<double, 2>> minimums(input.noParticles);
	std::vector<double> real_solutions(input.noParticles);
	Parameters* params = new Parameters[input.noParticles];
	std::vector<std::thread> particles(input.noParticles);
	
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
		positions[i].globalPosition = bestSolution;
	}
	std::barrier bar(input.threads);
	int common = 0;
	std::mutex mut;
	std::vector<NextMove> calcData;
	
	for (int j = 0; j < threadRanges.size(); j++)
	{
		calcData.emplace_back(threadRanges[j], input.iterations, j, params,
			positions, input.goalFunction);
		particles[j] = std::thread([&, j] {
			for (int c = 0; c < input.iterations; c++)
			{
				calcData[j].currentIteration = c;
				FindLocalMinimum(calcData[j], minimums, real_solutions);
				bar.arrive_and_wait();
				if (j == 0) {
					mut.lock();
					int index=std::min_element(real_solutions.begin(), real_solutions.end()) - real_solutions.begin();
					if(real_solutions[index]<input.goalFunction(bestSolution[0],bestSolution[1]))
						bestSolution=minimums[index];
					mut.unlock();
				}
				bar.arrive_and_wait();
				for (int d = threadRanges[j][0]; d < threadRanges[j][1]; d++, positions[d].globalPosition = bestSolution);
			}
			});
	}
	for (int j = 0; j < threadRanges.size(); j++)
	{
		particles[j].join();
	}
	SwarmOutputata output;
	for (int i = 0; i < input.noParticles; i++)
	{
		output.minimums.push_back(minimums[i]);
	}
	output.x = bestSolution[0];
	output.y = bestSolution[1];
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
	scope *= EPSILON_EXP;
	double number = -1.0;
	std::random_device rd;
	std::mt19937 gen{ rd() };
	std::normal_distribution<> dist(15000, 10000);
	while (number<0 || number>scope)
	{
		number = dist(rd);
	}
	return number/EPSILON_EXP+min_max[1];
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
	
	double newPositiveX = position->currentPosition[0] + velocity;
	double newNegativeX = position->currentPosition[0] - velocity;
	double newPositiveY = position->currentPosition[1] + velocity;
	double newNegativeY = position->currentPosition[1] - velocity;

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
	double localDistance = std::sqrt(std::pow(positions->bestLocalPosition[0] - currentPosition[0], 2) + std::pow(positions->bestLocalPosition[1] - currentPosition[1], 2));
	double globalDistance = std::sqrt(std::pow(positions->globalPosition[0] - currentPosition[0], 2) + std::pow(positions->globalPosition[1] - currentPosition[1], 2));
	positions->velocity = positions->velocity * parameters.w + RandomInThousand()/1000.0 * parameters.c1 * localDistance + RandomInThousand() / 1000.0 * parameters.c2 * globalDistance;
	currentPosition = FindBestDirection(positions);
	positions->currentPosition = currentPosition;
	double oldPositionGoalFunctionValue = positions->goalFunction(positions->bestLocalPosition[0], positions->bestLocalPosition[1]);
	double newPositionGoalFunctionValue = positions->goalFunction(currentPosition[0], currentPosition[1]);
	if (newPositionGoalFunctionValue < oldPositionGoalFunctionValue)
		positions->bestLocalPosition = currentPosition;
}

std::vector<std::array<int, 2>> CalculateThreadBounds(SwarmInputData* input)
{
	int noThreads = input->threads;
	int noParticles = input->noParticles;
	if (noThreads == 1)
	{
		return std::vector<std::array<int, 2>> { { 0, noParticles }};
	}
	int noPartitions = -1;
	if (noThreads == 0) //divice number of threads
	{
		noThreads = std::thread::hardware_concurrency();
	}
	if (noThreads % 2 == 1)
	{
		noThreads = std::thread::hardware_concurrency();
	}
	if (noParticles % 1 == 1)
	{
		input->noParticles += 1;
	}
	std::vector<std::array<int, 2>> output;
	int range = noParticles / noThreads;
	output.push_back({ 0,range });
	for (int i = 1; i < noThreads; i++)
	{
		output.push_back({ i * range,i * range + range });
	}
	return output;
}

void FindLocalMinimum(NextMove data,std::vector<std::array<double,2>>& minimums,
	std::vector<double>& real_solutions)
{
		for (int j = data.range[0]; j < data.range[1]; j++)
		{
			data.params[j] = CalculateParameters(j, data.noIterations);
			CalculateBestLocalPosition(data.positions, data.params[j]);
		}

		for (int j = data.range[0]; j < data.range[1]; j++)
		{
			minimums[j] = data.positions[j].bestLocalPosition;
			double x = minimums[j][0];
			double y = minimums[j][1];
			real_solutions[j] = data.fun(x, y);
		}
}

NextMove::NextMove(std::array<int, 2>range, int noIt, int currIt, Parameters* par, Positions* pos,
	std::function<double(double, double)> goalFun)
{
	this->range=range;
	noIterations=noIt;
	currentIteration=currIt;
	params=par;
	positions=pos;
	fun=goalFun;
}


