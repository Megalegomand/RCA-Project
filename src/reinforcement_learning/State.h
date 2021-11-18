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
	Point center_pt;

	//Siblings /children /connected states
	vector<State*> connected_states;

	//state status (current or not)
	bool current = false;

	//state reward of current state
	int reward = 0;

	// points properties
	int thickness = -1;

public:

State();
State(Mat* map, int x_coordinate_, int y_coordinate_); // vector <State*> connections );
void set_current_state(Mat *map, bool status);
void set_reward(int value);
void set_connected_states(vector<State*>child);
int get_reward();
Point get_center();
State get_current_state();
~State();

};

