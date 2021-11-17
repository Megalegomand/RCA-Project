#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

class State
{
	
private:
	// location of State in the envoriment
	int x_coordinate;
	int y_coordinate;

	//state status (current or not)

	bool current = false;


	// points properties
	int thickness = -1;

public:

State();
State(Mat* map, int x_coordinate_, int y_coordinate_); // vector <State *> connections );
void setCurrentState(Mat *map, bool status);
~State();



};

