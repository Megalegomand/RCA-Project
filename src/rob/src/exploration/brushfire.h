#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

enum class Neighbors
{
    Connected8,
    Connected4
};

class BrushFire
{
private:
    static Point neighbors_8[8];
    static Point neighbors_4[4];

    /**
     * @brief Construct a new Brushfire object. Private, since all
     * functions are static.
     */
    BrushFire();

    /**
     * @brief Checks if point is inside image range.
     * 
     * @param img Image
     * @param p Point to check for
     * @return true Is inside image
     * @return false Is not inside image
     */
    static bool in_range(Mat *img, Point p);

    /**
     * @brief Neighbors to array
     * 
     * @param neighbors Neighbors enum
     * @param[out] neighbors_array Output array
     * @param[out] neighbor_size Output array size
     */
    static void neighbors_array(Neighbors neighbors, Point **neighbor_array, int *neighbor_size);

public:
    /**
     * \defgroup brushfire
     * @brief Calculate a brushfire map, but creating rings around 
     * obstacles marked as black on the input image.
     * 
     * @param img Input image
     * @param neighbors Amount of neighbors, either edges or edges + 
     * corners
     * @param obs_color Obstacle color
     * @return Brushfire map as 16 bit, 1 channel Mat
     */
    static Mat brushfire(Mat *img, Neighbors neighbors, Vec3b obs_color);
    static Mat brushfire(Mat *img, Neighbors neighbors);
    /** @} */

    static Mat voronoi(Mat *bf_img, Neighbors neighbors);

    /**
     * @brief Converts a 16 bit brushfire image to a 255 bit displayable
     * format
     * 
     * @param bf_img 16-bit 1 channel image from brushfire()
     * @return Mat 3 channel image.
     */
    static Mat bf_to_display(Mat *bf_img);
    ~BrushFire();
};
