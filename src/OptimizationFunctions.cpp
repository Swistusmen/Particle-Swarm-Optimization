#include "OptimizationFunctions.h"

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
			solutions[j] = particles[j]->GetBestLocalPosition();
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
	for (int i = 0; i < input.noParticles; i++)
	{
		output.minimums.push_back({ solutions[i].first,solutions[i].second });
	}
	output.x = bestSolution.first;
	output.y = bestSolution.second;
	output.z = input.goalFunction(output.x, output.y);
	
	return output;
}
