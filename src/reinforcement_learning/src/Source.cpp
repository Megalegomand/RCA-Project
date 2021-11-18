#include <iostream>
#include <vector.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "State.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv) {

	
	// Loads in the map to be displayed
	cv::Mat map;
	cv::namedWindow("Map", cv::WINDOW_NORMAL);
	map = cv::imread("C:/Users/rasm4/Downloads/floor_plan.png"); // change this to change map
	
	State one(&map, 8, 9);
	State two(&map, 23, 9);
	State three(&map, 23, 28);
	State four(&map, 35, 38);
	State five(&map, 42, 24);
	State six(&map, 68, 24);
	State seven(&map, 42, 10);
	State eight(&map, 68, 10);
	State nine(&map, 92, 8);
	State ten(&map, 115, 20);
	State eleven(&map, 35, 60);
	State tvelve(&map, 101, 40);
	State thirteen(&map, 114, 62);
	State fourteen(&map, 68, 55);
	State fiveteen(&map, 78, 70);
	State sixteen(&map, 55, 75);
	State seventeen(&map, 8, 45);

	vector<State*> connnect_one ={&one, &two};
	vector<State*> connnect_two ={&one, &two};
	vector<State*> connnect_three ={&one, &two};
	vector<State*> connnect_four ={&one, &two};
	vector<State*> connnect_five ={&one, &two};
	vector<State*> connnect_six ={&one, &two};
	vector<State*> connnect_seven ={&one, &two};
	vector<State*> connnect_eight ={&one, &two};
	vector<State*> connnect_nine ={&one, &two};
	vector<State*> connnect_ten ={&one, &two};
	vector<State*> connnect_eleven ={&one, &two};
	vector<State*> connnect_tvelve ={&one, &two};
	vector<State*> connnect_thirteen ={&one, &two};
	vector<State*> connnect_fourteen ={&one, &two};
	vector<State*> connnect_fiveteen ={&one, &two};
	vector<State*> connnect_sixteen ={&one, &two};
	vector<State*> connnect_seventeen ={&one, &two};

	one.set_connected_states(&map,connect_one);
	two.set_connected_states(&map,connect_two);
	three.set_connected_states(&map,connect_three);
	four.set_connected_states(&map,connect_four);
	five.set_connected_states(&map,connect_five);
	six.set_connected_states(&map,connect_six);
	seven.set_connected_states(&map,connect_seven);
	eight.set_connected_states(&map,connect_eight);
	nine.set_connected_states(&map,connect_nine);
	ten.set_connected_states(&map,connect_ten);
	eleven.set_connected_states(&map,connect_eleven);
	tvelve.set_connected_states(&map,connect_tvelve);
	thirteen.set_connected_states(&map,connect_thirteen);
	fourteen.set_connected_states(&map,connect_fourteen);
	fiveteen.set_connected_states(&map,connect_fiveteen);
	sixteen.set_connected_states(&map,connect_sixteen);
	seventeen.set_connected_states(&map,connect_seventeen);

	ten.set_current_state(&map, true);
	ten.set_current_state(&map, false);
	ten.set_current_state(&map, true);

cv::imshow("Map", map);


	cv::waitKey(0);
	return 0;
}