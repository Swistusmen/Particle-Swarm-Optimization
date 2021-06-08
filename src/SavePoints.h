#include <fstream>
#include <vector>
#include <functional>
#include <array>
#include <string>

void SavePoints(std::vector<std::array<double, 2>> points, std::function<double(double, double)> fun,
	std::string filename)
{
	std::ofstream file;
	file.open("../../Viewer/"+filename);
	for (int i = 0; i < points.size(); i++)
	{
		file << points[i][0] << " " << points[i][1] << " " << fun(points[i][0], points[i][1]) << "\n";
	}
	file.close();
}