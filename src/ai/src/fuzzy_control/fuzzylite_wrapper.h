#include "fl/Headers.h"

using namespace fl;
using namespace std;

class FuzzyLiteWrapper
{
  private:
    Engine *engine;

    InputVariable *obs_distance_iv;
    InputVariable *obs_angle_iv;
    InputVariable *target_distance_iv;
    InputVariable *target_angle_iv;

    OutputVariable *velocity_ov;
    OutputVariable *turn_speed_ov;

    /**
     * @brief Initialize FuzzyLite engine with rules needed for
     * fuzzycontrol.
     */
    void init_engine();

  public:
    /**
     * @brief Construct a new FuzzyLite Wrapper object
     */
    FuzzyLiteWrapper();

    /**
     * @brief Update values
     *
     * @param[out] vel Output velocity. [m/s]
     * @param[out] turn_speed Output turn speed. [rad/s]
     * @param[in] obs_distance Distance to obstacle. [m]
     * @param[in] obs_angle Forward angle to obstacle. [rad]
     * @param[in] target_distance Distance to target. [m]
     * @param[in] target_angle Forward angle to target. [rad]
     */
    void update(float *vel, float *turn_speed, float obs_distance,
                float obs_angle, float target_distance, float target_angle);

    /**
     * @brief Destroy the FuzzyLite Wrapper object
     */
    ~FuzzyLiteWrapper();
};
