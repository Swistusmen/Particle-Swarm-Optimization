#include "OptimizationFunctions.h"

SwarmOutputata SwarmMultiThread(SwarmInputData input)
{
	std::vector<std::unique_ptr<Particle>> particles;
	std::vector<double*> minimums;
	std::vector<std::pair<double, double>> solutions;
	std::vector<double> real_solutions;
	const int noParticles = input.noParticles;
	const int n = input.iterations;
	for (int i = 0; i < noParticles; i++)
	{
		minimums.push_back(new double[4]{ 0.0,0.0,0.0,0.0 });
		particles.push_back(std::make_unique<Particle>(input.iterations, input.goalFunction, input.X, input.Y, minimums.back()));
		solutions.push_back({ minimums.back()[0], minimums.back()[1] });
		real_solutions.push_back(input.goalFunction(solutions.back().first, solutions.back().second));
	}

	auto bestSolution = solutions[std::min_element(real_solutions.begin(), real_solutions.end()) - real_solutions.begin()];

	for (int j = 0; j < noParticles; j++)
	{
		minimums[j][2] = bestSolution.first;
		minimums[j][3] = bestSolution.second;
	}


	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < noParticles; j++)
		{
			particles[j]->CalculateNextPositionThread(); //do zrownoleglenia
			solutions[j] = { minimums[j][0],minimums[j][1] };
			real_solutions[j] = input.goalFunction(solutions[j].first, solutions[j].second);
		}
		auto tempBestSolution = solutions[std::min_element(solutions.begin(), solutions.end()) - solutions.begin()];
		if (input.goalFunction(tempBestSolution.first, tempBestSolution.second) < input.goalFunction(bestSolution.first, bestSolution.second))
		{
			bestSolution = tempBestSolution;
			for (int j = 0; j < noParticles; j++) //do zrownoleglenia
			{
				particles[j]->SetBestGlobalPosition(bestSolution);
				minimums[j][2] = bestSolution.first;
				minimums[j][3] = bestSolution.second;
			}
		}
	}
	SwarmOutputata output;

	output.x = bestSolution.first;
	output.y = bestSolution.second;
	output.z = input.goalFunction(output.x, output.y);

	return output;
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
		particles.push_back(std::make_unique<Particle>(input.iterations,input.goalFunction, input.X, input.Y ));
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
