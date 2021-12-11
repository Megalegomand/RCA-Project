#include "envoriment.h"

using namespace std;
using namespace cv;

Envoriment::Envoriment()
{
}

Envoriment::Envoriment(Mat map)
{

	int rows = map.rows;
	int cols = map.cols;

	int white_val = -1;
	int black_val = 0;
	int red_val = -8;
	int green_val = 80;
	int blue_val = 200;

	for (int i = 0; i < rows; i++)
	{
		vector<State> state_temp;

		for (int y = 0; y < cols; y++)
		{
			vector<int> pixel = {(int)map.at<cv::Vec3b>(i, y)[0], (int)map.at<cv::Vec3b>(i, y)[1], (int)map.at<cv::Vec3b>(i, y)[2]};

			// if pixel is white
			if (pixel[0] == 255 & pixel[1] == 255 & pixel[2] == 255)
			{

				State white_state(i, y, white_val);
				white_state.set_color_val(255, 255, 255);
				state_temp.push_back(white_state);
			}
			// if pixel is black
			if (pixel[0] == 0 & pixel[1] == 0 & pixel[2] == 0)
			{
				;

				State black_state(i, y, black_val);
				black_state.set_color_val(0, 0, 0);
				state_temp.push_back(black_state);
			}
			// if pixel is red
			if (pixel[0] == 0 & pixel[1] == 0 & pixel[2] == 255)
			{

				State red_state(i, y, red_val);
				red_state.set_color_val(0, 0, 255);
				state_temp.push_back(red_state);
			}
			// if pixel is green
			if (pixel[0] == 0 & pixel[1] == 255 & pixel[2] == 0)
			{

				State green_state(i, y, green_val);
				green_state.set_color_val(0, 255, 0);
				state_temp.push_back(green_state);
			}
			// if pixel is blue
			if (pixel[0] == 255 & pixel[1] == 0 & pixel[2] == 0)
			{

				State blue_state(i, y, blue_val);
				blue_state.set_color_val(255, 0, 0);
				state_temp.push_back(blue_state);
			}
			//ROS_INFO("%i,%i",i,y);
		}
		envoriment.push_back(state_temp);
		state_temp.clear();
	}
	// ---- Set state relationship
	for (int i = 0; i < envoriment.size(); i++)
	{

		for (int y = 0; y < envoriment[0].size(); y++)
		{
			if (envoriment[i][y].get_reward() != black_val)
			{
				envoriment[i][y].set_connected_states(&envoriment[i - 1][y]); // Upper state
				envoriment[i][y].set_connected_states(&envoriment[i + 1][y]); // Lower state
				envoriment[i][y].set_connected_states(&envoriment[i][y - 1]); // Left state
				envoriment[i][y].set_connected_states(&envoriment[i][y + 1]); // Right state
			}
		}
	}
}

vector<vector<State>> *Envoriment::get_envoriment()
{
	return &envoriment;
}

State *Envoriment::get_state(int x, int y)
{
	return &envoriment[x][y];
}

void Envoriment::reset_map(Mat map)
{
	Vec3b red = {0, 0, 255};
	Vec3b green = {0, 255, 0};
	Vec3b blue = {255, 0, 0};

	for (int i = 0; i < envoriment.size(); i++)
	{
		for (int j = 0; j < envoriment[0].size(); j++)
		{ // if state was original a red, green or blue state
			envoriment[i][j].set_isVisted();
			envoriment[i][j].reset_VisitedCounter();

			if (envoriment[i][j].get_color_val() == red || envoriment[i][j].get_color_val() == green || envoriment[i][j].get_color_val() == blue)
			{

				int x = envoriment[i][j].get_location().first;
				int y = envoriment[i][j].get_location().second;

				Vec3b reset = map.at<Vec3b>(x, y);

				reset[0] = envoriment[i][j].get_color_val()[0];
				reset[1] = envoriment[i][j].get_color_val()[1];
				reset[2] = envoriment[i][j].get_color_val()[2];

				map.at<Vec3b>(x, y) = reset;
			}
		}
	}
}

void Envoriment::show_envoriment()
{
	for (auto i : envoriment)
	{
		for (auto y : i)
		{
			cout << y.get_reward();
		}
		cout << endl;
	}
}

Envoriment::~Envoriment()
{
}