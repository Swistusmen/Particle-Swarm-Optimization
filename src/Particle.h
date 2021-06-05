#include <tuple>
#include <cstdlib>
#include <functional>
#include <array>
#include <ctime>
#include <math.h>
#include <vector>
#include <numeric>
#include <limits>
#include <algorithm>
#include <random>
#include "Utilities.h"
//after all change it to class hierarchy- one class for common implementation, one for thread, one for 1 thread, and one for implementation

class Particle {
public:
	explicit Particle(int numberOfIterations, std::function< double( double,  double)> fun, std::array< double, 2> maxX, std::array< double,2> maxY);
	explicit Particle(int numberOfIterations, std::function<double(double, double)>fun, std::array<double, 2> maxX, std::array<double, 2> maxY, double* locations);

	void CalculateNextPosition();
	void CalculateNextPositionThread(); //function operates on raw memory
	std::pair< double,  double> GetBestPosition() {return bestPosition;};
	void SetBestGlobalPosition(std::pair<double, double> solution);

private:
	inline void RandomizeR();
	std::pair<double, double> GenerateStartingPositions(std::array< double,2> x, std::array< double,2> y);
	std::pair<double, double> FindBestDirection(double velocity);
private:
	double velocity;
	double rl, rg;
	std::pair< double,  double> currentPosition;
	std::pair<double, double> bestLocalPosition;
	std::pair< double,  double> bestPosition;
	double w; //is calculated accoridng to the https://www.matec-conferences.org/articles/matecconf/pdf/2016/26/matecconf_mmme2016_02019.pdf
	double c1, c2; //calulated in every iteration according to the Ratnaweera equation-https://web2.qatar.cmu.edu/~gdicaro/15382-Spring18/hw/hw3-files/pso-book-extract.pdf
	const double cmax = 2.5, cmin = 0.5; //according to to Ratnaweera
	double wmax = 0.9, wmin = 0.4; //accoring to menitoned method in w

	std::function< double( double,  double)> goalFunction;
	 int n=0; //current iteration
	 int T;//number of iterations

	std::array< double, 2> X;
	std::array< double, 2> Y;
	double* locations;

	std::random_device rd;
	std::mt19937 gen{ rd() };
};