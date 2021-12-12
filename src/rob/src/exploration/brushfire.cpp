#include "brushfire.h"

Point BrushFire::neighbors_8[8] = {
    Point(-1, -1),
    Point(0, -1),
    Point(1, -1),
    Point(1, 0),
    Point(1, 1),
    Point(0, 1),
    Point(-1, 1),
    Point(-1, 0)};

Point BrushFire::neighbors_4[4] = {
    Point(0, -1),
    Point(1, 0),
    Point(0, 1),
    Point(-1, 0)};

BrushFire::BrushFire() {}

bool BrushFire::in_range(Mat *img, Point p)
{
    return p.x >= 0 && p.y >= 0 && p.x < img->cols && p.y < img->rows;
}

void BrushFire::neighbors_array(Neighbors neighbors, Point **neighbor_array, int *neighbor_size)
{
    switch (neighbors)
    {
    case Neighbors::Connected8:
        *neighbor_array = neighbors_8;
        *neighbor_size = 8;
        break;
    case Neighbors::Connected4:
        *neighbor_array = neighbors_4;
        *neighbor_size = 4;
        break;
    default:
        throw "Neighbors not handled.";
        break;
    }
}

Mat BrushFire::brushfire(Mat *img, Neighbors neighbors, Vec3b obs_color)
{
    // Brushfire image
    // 1 channel 16 bit unsigned initialized with zeros
    Mat bf_img = Mat(img->rows, img->cols, CV_16UC1, Scalar(0));

    // List of pointers to bf_img for the current ring
    vector<Point> ring = vector<Point>();

    // Find all obstacles and add them to the rings list
    for (int x = 0; x < img->cols; x++)
    {
        for (int y = 0; y < img->rows; y++)
        {
            Point p(x, y);
            if (img->at<Vec3b>(p) == obs_color)
            {
                bf_img.at<short>(p) = 1;
                ring.push_back(p);
            }
        }
    }

    Point *neighbor_array;
    int neighbor_size;
    neighbors_array(neighbors, &neighbor_array, &neighbor_size);

    // Calculate each incrementing ring until no more rings.
    vector<Point> new_ring = vector<Point>();
    while (ring.size() > 0)
    {
        // Fill neighbors 
        for (Point p : ring)
        {
            int filled_neighbors = 0;
            for (int i = 0; i < neighbor_size; i++)
            {
                Point np = p + neighbor_array[i];
                if (in_range(&bf_img, np))
                {
                    if (bf_img.at<short>(np) == 0)
                    {
                        bf_img.at<short>(np) = bf_img.at<short>(p) + 1;
                        new_ring.push_back(np);
                        filled_neighbors ++;
                    }
                }
            }
        }
        ring = new_ring;
        new_ring.clear();
    }

    return bf_img;
}

Mat BrushFire::brushfire(Mat *img, Neighbors neighbors)
{
    return BrushFire::brushfire(img, neighbors, Vec3b(0, 0, 0));
}

Mat BrushFire::voronoi(Mat *bf_img, Neighbors neighbors)
{
    Mat v_img = Mat(bf_img->rows, bf_img->cols, CV_8UC1, Scalar(0));

    Point *neighbor_array;
    int neighbor_size;
    neighbors_array(neighbors, &neighbor_array, &neighbor_size);

    for (int x = 0; x < bf_img->cols; x++)
    {
        for (int y = 0; y < bf_img->rows; y++)
        {
            Point p(x, y);
            if (bf_img->at<short>(p) > 2)
            {
                int mag_x = 0;
                int mag_y = 0;
                int mag_xs = 0;
                int mag_ys = 0;

                for (int i = 0; i < neighbor_size; i++)
                {
                    Point np = p + neighbor_array[i];
                    if (in_range(bf_img, np))
                    {
                        mag_x += bf_img->at<short>(np) * neighbor_array[i].x;
                        mag_y += bf_img->at<short>(np) * neighbor_array[i].y;
                        mag_xs += abs(neighbor_array[i].x);
                        mag_ys += abs(neighbor_array[i].y);
                    }
                }
                mag_x = mag_x;// / mag_xs;
                mag_y = mag_y;// / mag_ys;

                v_img.at<uchar>(p) = (int) sqrt(mag_x * mag_x + mag_y * mag_y) * 64;
            }
        }
    }

    return v_img;
}

Mat BrushFire::bf_to_display(Mat *bf_img)
{
    Mat out = Mat(bf_img->rows, bf_img->cols, CV_8UC3);

    double max_val;
    minMaxIdx(*bf_img, NULL, &max_val, NULL, NULL);

    for (int x = 0; x < bf_img->cols; x++)
    {
        for (int y = 0; y < bf_img->rows; y++)
        {
            Point p(x, y);
            if (bf_img->at<short>(p) == 1)
            {
                out.at<Vec3b>(p) = Vec3b(0, 0, 0);
            }
            else if (bf_img->at<short>(p) > 1)
            {
                int normalized_val = bf_img->at<short>(p) / max_val * 255;
                out.at<Vec3b>(p) = Vec3b(255, normalized_val, normalized_val);
            }
        }
    }

    return out;
}

BrushFire::~BrushFire() {}
