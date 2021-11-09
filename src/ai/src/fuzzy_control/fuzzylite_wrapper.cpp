#include <math.h>
#include "fuzzylite_wrapper.h"

#define LIDAR_RANGE  (3*M_PI)/4

FuzzyLiteWrapper::FuzzyLiteWrapper()
{
    init_engine();

    string status;
    if (!engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status,
                        FL_AT);

    //obs_distance_iv = engine->getInputVariable("distance");
    obs_angle_iv = engine->getInputVariable("obs_angle");
    // target_distance_iv = engine->getInputVariable("angleSteering");
    // target_angle_iv = engine->getInputVariable("angleTarget");

    // TODO Implement velocity control
    // velocity_ov = engine->getOutputVariable("");
    turn_speed_ov = engine->getOutputVariable("turn_speed");
    ;
}

void FuzzyLiteWrapper::init_engine()
{
    engine = new Engine();
    engine->setName("TargetSteeringModule");
    engine->setDescription("");

    InputVariable *obs_angle = new InputVariable;
    obs_angle->setName("obs_angle");
    obs_angle->setDescription("");
    obs_angle->setEnabled(true);
    obs_angle->setRange(-LIDAR_RANGE, LIDAR_RANGE);
    obs_angle->setLockValueInRange(false);
    obs_angle->addTerm(new Ramp("left", -LIDAR_RANGE, 0.000));
    obs_angle->addTerm(new Ramp("right", 0.000, LIDAR_RANGE));
    engine->addInputVariable(obs_angle);

    OutputVariable *turn_speed = new OutputVariable;
    turn_speed->setName("turn_speed");
    turn_speed->setDescription("");
    turn_speed->setEnabled(true);
    turn_speed->setRange(-1.00, 1.00);
    turn_speed->setLockValueInRange(false);
    turn_speed->setAggregation(new Maximum);
    turn_speed->setDefuzzifier(new Centroid(100));
    turn_speed->setDefaultValue(fl::nan);
    turn_speed->setLockPreviousValue(false);
    turn_speed->addTerm(new Ramp("left", -0.10, 0.000));
    turn_speed->addTerm(new Ramp("right", 0.000, 0.10));
    engine->addOutputVariable(turn_speed);

    // OutputVariable *target_angle = new OutputVariable;
    // turn_speed->setName("target_angle");
    // turn_speed->setDescription("");
    // turn_speed->setEnabled(true);
    // turn_speed->setRange(-0.400, 0.400);
    // turn_speed->setLockValueInRange(false);
    // turn_speed->setAggregation(new Maximum);
    // turn_speed->setDefuzzifier(new Centroid(100));
    // turn_speed->setDefaultValue(fl::nan);
    // turn_speed->setLockPreviousValue(false);
    // turn_speed->addTerm(new Ramp("left", -0.400, 0.000));
    // turn_speed->addTerm(new Ramp("right", 0.000, 0.400));
    // engine->addOutputVariable(turn_speed);


    RuleBlock *mamdani = new RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setDescription("");
    mamdani->setEnabled(true);
    mamdani->setConjunction(fl::null);
    mamdani->setDisjunction(fl::null);
    mamdani->setImplication(new AlgebraicProduct);
    mamdani->setActivation(new General);
    mamdani->addRule(
        Rule::parse("if obs_angle is left then turn_speed is right", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is right then turn_speed is left", engine));
    engine->addRuleBlock(mamdani);
}

void FuzzyLiteWrapper::update(float *vel, float *turn_speed,
                              float obs_distance, float obs_angle,
                              float target_distance, float target_angle)
{
    // Update input values
    //obs_distance_iv->setValue(obs_distance);
    obs_angle_iv->setValue(obs_angle);

    engine->process();

    // Update output
    *vel = 0.5f; // velocity_ov->getValue();
    *turn_speed = turn_speed_ov->getValue();
}

FuzzyLiteWrapper::~FuzzyLiteWrapper()
{
}
