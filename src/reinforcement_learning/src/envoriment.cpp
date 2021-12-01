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

	int white_val = 0;
	int black_val = -1;
	int red_val = -5;
	int green_val = 1;
	int blue_val = 5;

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
				state_temp.push_back(white_state);

			}
			// if pixel is black
			if (pixel[0] == 0 & pixel[1] == 0 & pixel[2] == 0)
			{;

				State black_state(i, y, black_val);
				state_temp.push_back(black_state);

			}
			// if pixel is red
			if (pixel[0] == 0 & pixel[1] == 0 & pixel[2] == 255)
			{

				State red_state(i, y, red_val);
				state_temp.push_back(red_state);

			}
			// if pixel is green
			if (pixel[0] == 0 & pixel[1] == 255 & pixel[2] == 0)
			{

				State green_state(i, y, green_val);
				state_temp.push_back(green_state);

			}
			// if pixel is blue
			if (pixel[0] == 255 & pixel[1] == 0 & pixel[2] == 0)
			{

				State blue_state(i, y, blue_val);
				state_temp.push_back(blue_state);

			}
		}
		envoriment.push_back(state_temp);
		state_temp.clear();
	}
    // ---- Set state relationship
	for (int i = 0; i < envoriment.size(); i++)
	{

		for (int y = 0; y < envoriment.size(); y++)
		{
			envoriment[i][y].set_connected_states(&envoriment[i-1][y]); // Upper state
			envoriment[i][y].set_connected_states(&envoriment[i+1][y]); // Lower state
			envoriment[i][y].set_connected_states(&envoriment[i][y-1]); // Left state
			envoriment[i][y].set_connected_states(&envoriment[i-1][y+1]); // Right state
		}
	}
}

vector<vector<State>>* Envoriment::get_envoriment()
{
    return &envoriment;
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