class FuzzyLiteWrapper
{
  private:
    /* data */
  public:
    FuzzyLiteWrapper ();

    /**
     * @brief Update values
     *
     * @param[out] vel Output velocity. [m/s]
     * @param[out] dir Output turn speed. [rad/s]
     * @param[in] obs_dist Distance to obstacle. [m]
     * @param[in] obs_angle Forward angle to obstacle. [rad]
     * @param[in] target_dist Distance to target. [m]
     * @param[in] target_angle Forward angle to target. [rad]
     */
    void update (float *vel, float *dir, float obs_dist, float obs_angle,
                 float target_dist, float target_angle);

    ~FuzzyLiteWrapper ();
};
