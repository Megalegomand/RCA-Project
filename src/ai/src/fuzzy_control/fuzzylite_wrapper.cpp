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

    obs_distance_iv = engine->getInputVariable("obs_distance");
    obs_angle_iv = engine->getInputVariable("obs_angle");
    //target_distance_iv = engine->getInputVariable("angleSteering");
    //target_angle_iv = engine->getInputVariable("angleTarget");

    velocity_ov = engine->getOutputVariable("velocity");
    turn_speed_ov = engine->getOutputVariable("turn_speed");
}

void FuzzyLiteWrapper::init_engine()
{
    engine = new Engine();
    engine->setName("TargetSteeringModule");
    engine->setDescription("");

    // Input variables for Obstacles Avoidance ------------------------

    InputVariable *obs_angle = new InputVariable;
    obs_angle->setName("obs_angle");
    obs_angle->setDescription("");
    obs_angle->setEnabled(true);
    obs_angle->setRange(-LIDAR_RANGE, LIDAR_RANGE);
    obs_angle->setLockValueInRange(false);
    obs_angle->addTerm(new Ramp("far_left", -M_PI, -LIDAR_RANGE));
    obs_angle->addTerm(new Triangle("left", -M_PI,-M_PI/8,-0.200 ));
    obs_angle->addTerm(new Triangle("center", -0.200, 0.000, 0.200));
    obs_angle->addTerm(new Triangle("right", 0.200, M_PI/8, M_PI));
    obs_angle->addTerm(new Ramp("far_right", M_PI, LIDAR_RANGE));
    engine->addInputVariable(obs_angle);

    InputVariable *obs_distance = new InputVariable;
    obs_distance->setName("obs_distance");
    obs_distance->setDescription("");
    obs_distance->setEnabled(true);
    obs_distance->setRange(0.000,10.000 );
    obs_distance->setLockValueInRange(false);
    obs_distance->addTerm(new Ramp("close", 1.500, 0.000));
    obs_distance->addTerm(new Triangle("medium", 1.500, 3.000, 5.000));
    obs_distance->addTerm(new Ramp("far", 5.000, 10.000));
    engine->addInputVariable(obs_distance);

    // Input variables for Target Steering --------------------------

    // InputVariable *angleTarget = new InputVariable;
    // angleTarget->setName("angleTarget");
    // angleTarget->setDescription("");
    // angleTarget->setEnabled(true);
    // angleTarget->setRange(-LIDAR_RANGE, LIDAR_RANGE);
    // angleTarget->setLockValueInRange(false);
    // angleTarget->setLockPreviousValue(false);
    // angleTarget->addTerm(new Ramp("left", 0.000, -LIDAR_RANGE));
    // angleTarget->addTerm(new Triangle("center", -0.500, 0.000, 0.500));
    // angleTarget->addTerm(new Ramp("right", 0.000, LIDAR_RANGE));
    // engine->addInputVariable(angleTarget);

    // InputVariable *angleSteering = new InputVariable;
    // angleSteering->setName("angleSteering");
    // angleSteering->setDescription("");
    // angleSteering->setEnabled(true);
    // angleSteering->setRange(-LIDAR_RANGE, LIDAR_RANGE);
    // angleSteering->setLockValueInRange(false);
    // angleSteering->setLockPreviousValue(false);
    // angleSteering->addTerm(new Ramp("left", 0.000, -LIDAR_RANGE));
    // angleSteering->addTerm(new Triangle("center", -0.500, 0.000, 0.500));
    // angleSteering->addTerm(new Ramp("right", 0.000, LIDAR_RANGE));
    // engine->addInputVariable(angleSteering);

    // Output variables -------------------------------------------

    OutputVariable *turn_speed = new OutputVariable;
    turn_speed->setName("turn_speed");
    turn_speed->setDescription("");
    turn_speed->setEnabled(true);
    turn_speed->setRange(-0.40, 0.40);
    turn_speed->setLockValueInRange(false);
    turn_speed->setAggregation(new Maximum);
    turn_speed->setDefuzzifier(new Centroid(100));
    turn_speed->setDefaultValue(fl::nan);
    turn_speed->setLockPreviousValue(false);
    turn_speed->addTerm(new Ramp("fast_left", -0.25, -0.40));
    turn_speed->addTerm(new Triangle("left",-0.25,-0.20,-0.10 ));
    turn_speed->addTerm(new Triangle("center", -0.10, 0.00, 0.10));
    turn_speed->addTerm(new Triangle("right", 0.10, 0.20, 0.25));
    turn_speed->addTerm(new Ramp("fast_right", 0.25, 0.40));
    engine->addOutputVariable(turn_speed);

    OutputVariable *velocity = new OutputVariable;
    velocity->setName("velocity");
    velocity->setDescription("");
    velocity->setEnabled(true);
    velocity->setRange(0, 1.40);
    velocity->setLockValueInRange(false);
    velocity->setAggregation(new AlgebraicProduct); // before maximum
    velocity->setDefuzzifier(new Centroid(100));
    velocity->setDefaultValue(fl::nan);
    velocity->setLockPreviousValue(false);
    velocity->addTerm(new Ramp("slow", 0.200, 0.000));
    velocity->addTerm(new Triangle("medium", 0.20, 0.400, 0.800));
    velocity->addTerm(new Ramp("fast", 0.600, 1.30));
    engine->addOutputVariable(velocity);

    RuleBlock *mamdani = new RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setDescription("");
    mamdani->setEnabled(true);
    mamdani->setConjunction(new AlgebraicProduct); // before minimum
    mamdani->setDisjunction(fl::null);
    mamdani->setImplication(new AlgebraicProduct);
    mamdani->setActivation(new General);

    //Ruleblock for turn_speed -------------------------------------------------------------------------------

    mamdani->addRule(
        Rule::parse("if obs_angle is far_left and obs_distance is close then turn_speed is right", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is far_left and obs_distance is medium then turn_speed is center", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is far_left and obs_distance is far then turn_speed is center", engine));
    
    mamdani->addRule(
        Rule::parse("if obs_angle is left and obs_distance is close then turn_speed is fast_right", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is left and obs_distance is medium then turn_speed is right", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is left and obs_distance is far then turn_speed is center", engine));
    
    mamdani->addRule(
        Rule::parse("if obs_angle is center and obs_distance is close then turn_speed is fast_right", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is center and obs_distance is medium then turn_speed is right", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is center and obs_distance is far then turn_speed is center", engine));

    mamdani->addRule(
        Rule::parse("if obs_angle is right and obs_distance is close then turn_speed is fast_left", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is right and obs_distance is medium then turn_speed is left", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is right and obs_distance is far then turn_speed is center", engine));
    
    mamdani->addRule(
        Rule::parse("if obs_angle is far_right and obs_distance is close then turn_speed is left", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is far_right and obs_distance is medium then turn_speed is center", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is far_right and obs_distance is far then turn_speed is center", engine));
    
    //Ruleblock for velocity -------------------------------------------------------------------------------

    mamdani->addRule(
        Rule::parse("if obs_angle is far_left and obs_distance is close then velocity is slow", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is far_left and obs_distance is medium then velocity is fast", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is far_left and obs_distance is far then velocity is fast", engine));
    
    mamdani->addRule(
        Rule::parse("if obs_angle is left and obs_distance is close then velocity is slow", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is left and obs_distance is medium then velocity is medium", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is left and obs_distance is far then velocity is fast", engine));
    
    mamdani->addRule(
        Rule::parse("if obs_angle is center and obs_distance is close then velocity is slow", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is center and obs_distance is medium then velocity is slow", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is center and obs_distance is far then velocity is fast", engine));

    mamdani->addRule(
        Rule::parse("if obs_angle is right and obs_distance is close then velocity is slow", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is right and obs_distance is medium then velocity is medium", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is right and obs_distance is far then velocity is fast", engine));
    
    mamdani->addRule(
        Rule::parse("if obs_angle is far_right and obs_distance is close then velocity is slow", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is far_right and obs_distance is medium then velocity is fast", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is far_right and obs_distance is far then velocity is fast", engine));
    engine->addRuleBlock(mamdani);
}

void FuzzyLiteWrapper::update(float *vel, float *turn_speed,
                              float obs_distance, float obs_angle,
                              float target_distance, float target_angle)
{
    // Update input values
    obs_distance_iv->setValue(obs_distance);
    obs_angle_iv->setValue(obs_angle);
    //target_distance_iv->setValue();
    //target_angle_iv->setValue();

    //Point(20,3); // Goal

    engine->process();

    // Update output
    *vel = velocity_ov->getValue();
    *turn_speed = turn_speed_ov->getValue();
}

FuzzyLiteWrapper::~FuzzyLiteWrapper()
{
}
