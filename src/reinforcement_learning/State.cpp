#include "State.h"

State::State()
{
}

State::State(Mat* map, int x_coordinate_, int y_coordinate_) // vector <State*> connections)
{
	x_coordinate = x_coordinate_;
	y_coordinate = y_coordinate_;

	Point p1(x_coordinate, y_coordinate);
	Point p2(x_coordinate + 2, y_coordinate + 2);

	rectangle(*map, p1, p2,
		Scalar(0, 255, 0),
		thickness, LINE_8);
}

void State::setCurrentState(Mat *map, bool status) 
{
	if (status == true)
	{
		cv::Point xy;
		xy.x = x_coordinate + 1; // the +1 makes sure it is placed in the middle of the square
		xy.y = y_coordinate + 1; // the +1 makes sure it is placed in the middle of the square

		map->at<Vec3b>(xy) = { 0,0,255 };
		current = true;
	}
	else if (status == false)
	{
		cv::Point xy;
		xy.x = x_coordinate + 1; // the +1 makes sure it is placed in the middle of the square
		xy.y = y_coordinate + 1; // the +1 makes sure it is placed in the middle of the square

		map->at<Vec3b>(xy) = { 0,255,0 };
		current = false;
	}
	

}

State::~State()
{
}
