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
	auto threadRanges = CalculateThreadBounds(&input);
	Positions* positions= new Positions[input.noParticles];
	std::vector<std::array<double, 2>> minimums(input.noParticles);
	std::vector<double> real_solutions(input.noParticles);
	Parameters* params = new Parameters[input.noParticles];
	std::vector<std::thread> particles(input.noParticles);
	std::mutex* mutexes = new std::mutex;
	int* jobDone = new int[input.noParticles]{ -1 };
	ThreadCommon tCommon;
	tCommon.mainThread = std::this_thread::get_id();
	tCommon.sorterID = std::this_thread::get_id();
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
	
	for (int j = 0; j < threadRanges.size(); j++)
	{
		NextMove next(params, positions, input, &bestSolution, &minimums, &real_solutions
			, threadRanges[j], mutexes,jobDone,j,&tCommon);
		particles[j] = std::thread(CalculateNextMove, next );
	}
	for (int j = 0; j < threadRanges.size(); j++)
	{
		particles[j].join();
	}
	/*
	for (int i = 0; i < input.iterations; i++)
	{
		for (int j = 0; j < threadRanges.size(); j++)
		{
			particles[j] = std::thread(CalculateNextMove, threadRanges[j], positions, params,j, input.iterations);
		}
		for (int j = 0; j < threadRanges.size(); j++)
		{
			particles[j].join();
		}
		
		for (int j = 0; j < input.noParticles; j++)
		{
			minimums[j] = positions[j].bestLocalPosition;
			real_solutions[j] = input.goalFunction(minimums[j][0], minimums[j][1]);
		}
		
		auto tempBestSolution = minimums[std::min_element(minimums.begin(), minimums.end()) - minimums.begin()];
		if (input.goalFunction(tempBestSolution[0], tempBestSolution[1]) < input.goalFunction(bestSolution[0], bestSolution[1]))
		{
			bestSolution = tempBestSolution;
			for (int j = 0; j < input.noParticles; j++)
			{
				positions[j].globalPosition=bestSolution;
			}
		}
	
	}*/
	SwarmOutputata output;
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
/*
void CalculateNextMove(std::array<int, 2> range, Positions* positions, Parameters* params, int iteration, int noIterations)
{
	for (int i = range[0]; i < range[1]; i++)
	{
		params[i] = CalculateParameters(iteration, noIterations);
		CalculateBestLocalPosition(&positions[i], params[i]);
	}
}
*/
void CalculateNextMove(NextMove next)
{
	std::cout << std::this_thread::get_id() << std::endl;
	int noIterations = next.input.iterations;
	for (int i = 0; i < noIterations; i++)
	{
		for (int j = next.range[0]; j < next.range[1]; j++)
		{
			next.params[j] = CalculateParameters(j, noIterations);
			CalculateBestLocalPosition(&next.positions[j], next.params[j]);
		}

		for (int j = next.range[0]; j < next.range[1]; j++)
		{
			next.minimums->operator[](j) = next.positions[j].bestLocalPosition;
			double x = (next.minimums->operator[](j))[0];
			double y = (next.minimums->operator[](j))[0];
			next.real_solutions->at(j) = next.input.goalFunction(x, y);
		}
		next.isJobDone[next.number] = -1;
		next.mymut->lock();
		if (next.tCommon->sorterID == next.tCommon->mainThread) {
			next.tCommon->sorterID = std::this_thread::get_id();
		}
		next.isJobDone[next.number] = i;
		next.mymut->unlock();
		std::array<double, 2> tempBestSolution;
		while (true)
		{
			next.mymut->lock();
			std::cout << std::this_thread::get_id()<<" "<<i << std::endl;
			if (std::this_thread::get_id() == next.tCommon->sorterID)
			{
				bool doAllThreadsEned = true;
				for (int j = 0; j < next.input.threads; j++)
				{
					std::cout << next.isJobDone[j] << " ";
					if (next.isJobDone[j] != i)
						doAllThreadsEned = false;
				}
				std::cout << std::endl;
				if (doAllThreadsEned == true) {
					std::cout << "inside\n";
					tempBestSolution = next.minimums->operator[](std::min_element(next.minimums->begin(), next.minimums->end()) - next.minimums->begin());
					if (next.input.goalFunction(tempBestSolution[0], tempBestSolution[1]) < next.input.goalFunction(next.bestSolution->operator[](0), next.bestSolution->operator[](1)))
					{
						*next.bestSolution = tempBestSolution;
					}
					next.tCommon->readyToCalc = true;
					break;
				}
			}
			if (next.tCommon->readyToCalc == true)
				break;
			next.mymut->unlock();
		}
		next.mymut->unlock();
		
		
		for (int j = next.range[0]; j < next.range[1]; j++)
		{
			next.positions[j].globalPosition = *next.bestSolution;
		}

		next.mymut->lock();
		if (std::this_thread::get_id() == next.tCommon->sorterID)
		{
			next.tCommon->sorterID = next.tCommon->mainThread;
			next.tCommon->readyToCalc = false;
		}
		next.mymut->unlock();
	}
}

NextMove::NextMove(Parameters* params, Positions* pos, SwarmInputData inp, std::array<double, 2>* sol,
	std::vector<std::array<double, 2>>* mins, std::vector<double>* rsol,std::array<int,2> r,
	std::mutex* m, int* tabOfJobs,int number,ThreadCommon* tcom)
{
	positions = pos;
	this->params = params;
	input = inp;
	bestSolution = sol;
	minimums = mins;
	real_solutions = rsol;
	range = r;
	mymut = m;
	this->isJobDone = tabOfJobs;
	this->number = number;
	tCommon = tcom;
}

