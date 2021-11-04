#include "fl/Headers.h"
#include "ros/ros.h"
#include "ros/package.h"
#include <ros/console.h>
#include <string>
#include "std_msgs/String.h"

#include <sstream>

using namespace std;
using namespace fl;

int main(int _argc, char **_argv)
{
    Engine *engine = new Engine;
    engine->setName("TargetSteeringModule");
    engine->setDescription("");

    InputVariable *angleSteering = new InputVariable;
    angleSteering->setName("angleSteering");
    angleSteering->setDescription("");
    angleSteering->setEnabled(true);
    angleSteering->setRange(-3.1415, 3.1415);
    angleSteering->setLockValueInRange(false);
    angleSteering->addTerm(new Gaussian("z", 0.1, 0));
    angleSteering->addTerm(new Bell("PM", 0.35, 4, 1.27));
    angleSteering->addTerm(new Sigmoid("PB", 16, 0.15));
    angleSteering->addTerm(new Bell("NM", 0.35, 4, -1.27));
    angleSteering->addTerm(new Sigmoid("NB", -16, -0.15));

    InputVariable *angleTarget = new InputVariable;
    angleTarget->setName("angleTarget");
    angleTarget->setDescription("");
    angleTarget->setEnabled(true);
    angleTarget->setRange(-3.1415, 3.1415);
    angleTarget->setLockValueInRange(false);
    angleSteering->addTerm(new Gaussian("z", 0.1, 0));
    angleSteering->addTerm(new Bell("PM", 0.35, 4, 1.27));
    angleSteering->addTerm(new Sigmoid("PB", 16, 0.15));
    angleSteering->addTerm(new Bell("NM", 0.35, 4, -1.27));
    angleSteering->addTerm(new Sigmoid("NB", -16, -0.15));

    OutputVariable *SteeringVar1 = new OutputVariable;
    SteeringVar1->setName("SteeringVar1");
    SteeringVar1->setDescription("");
    SteeringVar1->setEnabled(true);
    SteeringVar1->setRange(-1.15, 1.15);
    SteeringVar1->setLockValueInRange(false);
    SteeringVar1->setAggregation(new Maximum);
    SteeringVar1->setDefuzzifier(new Centroid(100));
    SteeringVar1->setDefaultValue(fl::nan);
    SteeringVar1->setLockPreviousValue(false);
    SteeringVar1->addTerm(new SigmoidDifference("NM", 24.5, -0.74, 52, -0.1));
    SteeringVar1->addTerm(new Bell("Z", 0.1, 2, 0));
    SteeringVar1->addTerm(new SigmoidDifference("PM", 24.5, 0.74, 52, 0.1));
    SteeringVar1->addTerm(new Sigmoid("PB", 18, 0.75));
    SteeringVar1->addTerm(new Sigmoid("NB", -18, -0.75));

    RuleBlock *mamdani1 = new RuleBlock;
    mamdani1->setName("mamdani1");
    mamdani1->setDescription("");
    mamdani1->setEnabled(true);
    mamdani1->setConjunction(fl::null);
    mamdani1->setDisjunction(fl::null);
    mamdani1->setImplication(new AlgebraicProduct);
    mamdani1->setActivation(new General);
    mamdani1->setActivation(new General);
    mamdani1->addRule(Rule::parse("if angleSteering is NB and angleTarget is NB then SteeringVar1 is Z", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is NB and angleTarget is NM then SteeringVar1 is Z", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is NB and angleTarget is Z then SteeringVar1 is PM", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is NB and angleTarget is PM then SteeringVar1 is PB ", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is NB and angleTarget is PB then SteeringVar1 is PB ", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is NM and angleTarget is NB then SteeringVar1 is Z", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is NM and angleTarget is NM then SteeringVar1 is Z", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is NM and angleTarget is Z then SteeringVar1 is Z", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is NM and angleTarget is PM then SteeringVar1 is PB", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is NM and angleTarget is PB then SteeringVar1 is PB", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is Z and angleTarget is NB then SteeringVar1 is NM", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is Z and angleTarget is NM then SteeringVar1 is NM", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is Z and angleTarget is Z then SteeringVar1 is Z", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is Z and angleTarget is PM then SteeringVar1 is PM", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is Z and angleTarget is PB then SteeringVar1 is PM", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is PM and angleTarget is NB then SteeringVar1 is NB", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is PM and angleTarget is NM then SteeringVar1 is NM", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is PM and angleTarget is Z then SteeringVar1 is Z", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is PM and angleTarget is PM then SteeringVar1 is Z", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is PM and angleTarget is PB then SteeringVar1 is Z ", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is PB and angleTarget is NB then SteeringVar1 is NB", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is PB and angleTarget is NM then SteeringVar1 is NM", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is PB and angleTarget is Z then SteeringVar1 is NM", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is PB and angleTarget is PM then SteeringVar1 is Z ", engine));
    mamdani1->addRule(Rule::parse("if angleSteering is PB and angleTarget is PB then SteeringVar1 is Z", engine));
    engine->addRuleBlock(mamdani1);

    InputVariable *distance = new InputVariable;
    distance->setName("distance");
    distance->setDescription("");
    distance->setEnabled(true);
    distance->setRange(0, 80);
    distance->setLockValueInRange(false);
    distance->addTerm(new ZShape("Z", 0.3, 20));
    distance->addTerm(new SigmoidDifference("S", 0.5, 8, 2, 18));
    distance->addTerm(new SShape("B", 16, 20));

    InputVariable *angleToObstacle = new InputVariable;
    angleToObstacle->setName("angleToObstacle");
    angleToObstacle->setDescription("");
    angleToObstacle->setEnabled(true);
    angleToObstacle->setRange(-3.1415, 3.1415);
    angleToObstacle->setLockValueInRange(false);
    angleToObstacle->addTerm(new ZShape("NM", -1.4, -0.5));
    angleToObstacle->addTerm(new ZShape("Z", 0.3, 20));
    angleToObstacle->addTerm(new SShape("PM", 0.5, 1.4));
    angleToObstacle->addTerm(new SShape("NS", 16, 20));
    angleToObstacle->addTerm(new Gaussian("PS", 0.3, 0.6));

    OutputVariable *SteeringVar2 = new OutputVariable;
    SteeringVar2->setName("SteeringVar2");
    SteeringVar2->setDescription("");
    SteeringVar2->setEnabled(true);
    SteeringVar2->setRange(-0.9, 0.9);
    SteeringVar2->setLockValueInRange(false);
    SteeringVar2->setAggregation(new Maximum);
    SteeringVar2->setDefuzzifier(new Centroid(100));
    SteeringVar2->setDefaultValue(fl::nan);
    SteeringVar2->setLockPreviousValue(false);
    SteeringVar2->addTerm(new ZShape("NM", -0.6, -0.3));
    SteeringVar2->addTerm(new Gaussian("Z", 0.05, 0));
    SteeringVar2->addTerm(new SigmoidDifference("PS", 88, 0.06, 35, 0.45));
    SteeringVar2->addTerm(new SShape("PM", 0.3, 0.6));
    SteeringVar2->addTerm(new SigmoidDifference("NS", 35, -0.45, 88, -0.06));

    RuleBlock *mamdani2 = new RuleBlock;
    mamdani2->setName("mamdani2");
    mamdani2->setDescription("");
    mamdani2->setEnabled(true);
    mamdani2->setConjunction(fl::null);
    mamdani2->setDisjunction(fl::null);
    mamdani2->setImplication(new AlgebraicProduct);
    mamdani2->setActivation(new General);
    mamdani2->setActivation(new General);
    mamdani2->addRule(Rule::parse("if distance is Z and angleToObstacle is NM then SteeringVar2 is PM", engine));
    mamdani2->addRule(Rule::parse("if distance is Z and angleToObstacle is NS then SteeringVar2 is PM", engine));
    mamdani2->addRule(Rule::parse("if distance is Z and angleToObstacle is Z then SteeringVar2 is NM", engine));
    mamdani2->addRule(Rule::parse("if distance is Z and angleToObstacle is PS then SteeringVar2 is NM", engine));
    mamdani2->addRule(Rule::parse("if distance is Z and angleToObstacle is PM then SteeringVar2 is NM", engine));
    mamdani2->addRule(Rule::parse("if distance is S and angleToObstacle is NM then SteeringVar2 is PM", engine));
    mamdani2->addRule(Rule::parse("if distance is S and angleToObstacle is NS then SteeringVar2 is PS", engine));
    mamdani2->addRule(Rule::parse("if distance is S and angleToObstacle is Z then SteeringVar2 is NM", engine));
    mamdani2->addRule(Rule::parse("if distance is S and angleToObstacle is PS then SteeringVar2 is NM", engine));
    mamdani2->addRule(Rule::parse("if distance is S and angleToObstacle is PM then SteeringVar2 is NM", engine));
    mamdani2->addRule(Rule::parse("if distance is B and angleToObstacle is NM then SteeringVar2 is Z", engine));
    mamdani2->addRule(Rule::parse("if distance is B and angleToObstacle is NS then SteeringVar2 is Z", engine));
    mamdani2->addRule(Rule::parse("if distance is B and angleToObstacle is Z then SteeringVar2 is Z", engine));
    mamdani2->addRule(Rule::parse("if distance is B and angleToObstacle is PS then SteeringVar2 is Z", engine));
    mamdani2->addRule(Rule::parse("if distance is B and angleToObstacle is PM then SteeringVar2 is Z", engine));
    engine->addRuleBlock(mamdani2);

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

    InputVariable *obstacle = engine->getInputVariable("angleSteering");
    OutputVariable *steer = engine->getOutputVariable("SteeringVar1");

    for (int i = 0; i <= 50; ++i)
    {
        scalar location = obstacle->getMinimum() + i * (obstacle->range() / 50);
        obstacle->setValue(location);
        engine->process();
        FL_LOG("obstacle.input = " << Op::str(location) << " => "
                                   << "steer.output = " << Op::str(steer->getValue()));
    }
}