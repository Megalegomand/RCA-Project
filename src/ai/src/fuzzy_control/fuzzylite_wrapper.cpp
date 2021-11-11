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
    target_distance_iv = engine->getInputVariable("target_distance");
    target_angle_iv = engine->getInputVariable("target_angle");

    velocity_ov = engine->getOutputVariable("velocity");
    turn_speed_ov = engine->getOutputVariable("turn_speed");
}

void FuzzyLiteWrapper::init_engine()
{
    engine = new Engine();
    engine->setName("TargetSteeringModule");
    engine->setDescription("");

    // Input variables for Obstacles Avoidance ---------------------------------

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

    // Input variables for Target Steering -----------------------------------

    InputVariable *target_distance = new InputVariable;
     target_distance->setName("target_distance");
     target_distance->setDescription("");
     target_distance->setEnabled(true);
     target_distance->setRange(-LIDAR_RANGE, LIDAR_RANGE);
     target_distance->setLockValueInRange(false);
     target_distance->addTerm(new Spike("finish", 0.000, 0.200));
     target_distance->addTerm(new Ramp("close", 1.500, 0.200));
     target_distance->addTerm(new Triangle("medium", 1.500, 3.000, 5.000));
     target_distance->addTerm(new Ramp("far", 5.000, 10.000));
    engine->addInputVariable(target_distance);

    InputVariable *target_angle = new InputVariable;
     target_angle->setName("target_angle");
     target_angle->setDescription("");
     target_angle->setEnabled(true);
     target_angle->setRange(-LIDAR_RANGE, LIDAR_RANGE);
     target_angle->setLockValueInRange(false);
     target_angle->addTerm(new Ramp("left", -0.200, -LIDAR_RANGE));
     target_angle->addTerm(new Triangle("center", -0.200, 0.000, 0.200));
     target_angle->addTerm(new Ramp("right", 0.200, LIDAR_RANGE));
    engine->addInputVariable(target_angle);

    // Output variables ----------------------------------------------------

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
    velocity->setAggregation(new Maximum); // before maximum
    velocity->setDefuzzifier(new Centroid(100));
    velocity->setDefaultValue(fl::nan);
    velocity->setLockPreviousValue(false);
    velocity->addTerm(new Spike("stop", 0.0, 0.200));
    velocity->addTerm(new Ramp("slow", 0.200, 0.000));
    velocity->addTerm(new Triangle("medium", 0.20, 0.400, 0.600));
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

    //Ruleblock for Obstacles Avoidance and turn_speed -------------------------------------------------------------------------------

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
    
    //Ruleblock for Obstacles Avoidance and velocity -------------------------------------------------------------------------------

    mamdani->addRule(
        Rule::parse("if obs_angle is far_left and obs_distance is close then velocity is medium", engine));
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
        Rule::parse("if obs_angle is far_right and obs_distance is close then velocity is medium", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is far_right and obs_distance is medium then velocity is fast", engine));
    mamdani->addRule(
        Rule::parse("if obs_angle is far_right and obs_distance is far then velocity is fast", engine));
    
    //Ruleblock for Target Steering -------------------------------------------------------------------------------

    mamdani->addRule(
        Rule::parse("if target_distance is finish and target_angle is left then turn_speed is fast_right and velocity is stop", engine));
    mamdani->addRule(
        Rule::parse("if target_distance is finish and target_angle is center then turn_speed is center and velocity is stop", engine));
    mamdani->addRule(
        Rule::parse("if target_distance is finish and target_angle is right then turn_speed is fast_left and velocity is stop", engine));
    
    mamdani->addRule(
        Rule::parse("if target_distance is close and target_angle is left then turn_speed is fast_right and velocity is slow", engine));
    mamdani->addRule(
        Rule::parse("if target_distance is close and target_angle is center then turn_speed is center and velocity is slow", engine));
    mamdani->addRule(
        Rule::parse("if target_distance is close and target_angle is right then turn_speed is fast_left and velocity is slow", engine));

    mamdani->addRule(
        Rule::parse("if target_distance is medium and target_angle is left then turn_speed is right and velocity is medium", engine));
    mamdani->addRule(
        Rule::parse("if target_distance is medium and target_angle is center then turn_speed is center and velocity is slow", engine));
    mamdani->addRule(
        Rule::parse("if target_distance is medium and target_angle is right then turn_speed is left and velocity is medium", engine));

    mamdani->addRule(
        Rule::parse("if target_distance is far and target_angle is left then turn_speed is right and velocity is fast", engine));
    mamdani->addRule(
        Rule::parse("if target_distance is far and target_angle is center then turn_speed is center and velocity is fast", engine));
    mamdani->addRule(
        Rule::parse("if target_distance is far and target_angle is right then turn_speed is left and velocity is fast", engine));


    engine->addRuleBlock(mamdani);
}

void FuzzyLiteWrapper::update(float *vel, float *turn_speed,
                              float obs_distance, float obs_angle,
                              float target_distance, float target_angle)
{
    // Update input values
    obs_distance_iv->setValue(obs_distance);
    obs_angle_iv->setValue(obs_angle);
    target_distance_iv->setValue(target_distance);
    target_angle_iv->setValue(target_angle);

    //Point(20,3); // Goal

    engine->process();

    // Update output
    *vel = velocity_ov->getValue();
    *turn_speed = turn_speed_ov->getValue();
}

FuzzyLiteWrapper::~FuzzyLiteWrapper()
{
}
