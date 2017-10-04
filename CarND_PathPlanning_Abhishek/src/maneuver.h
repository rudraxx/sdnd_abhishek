#include <vector>

using namespace std;

class Maneuver{
public:

	curr_px;
	curr_py;
	curr_vx;
	curr_vy;
	
	Maneuver(double px, double vx);

	vector<vector<double> > trajectory(); 
}