#include "RobotModel.hpp"

const RobotModel RobotModel2015 = []() {
    RobotModel model;
    model.WheelRadius = 0.02856;
    // note: wheels are numbered clockwise, starting with the top-right
    // TODO(ashaw596): Check angles.
    model.WheelAngles = {
        DegreesToRadians(38), DegreesToRadians(315), DegreesToRadians(225),
        DegreesToRadians(142),
    };
    model.WheelDist = 0.0798576;

    model.DutyCycleMultiplier = 9;  // TODO: tune this value

    model.recalculateBotToWheel();

    return model;
}();


const RobotModel RobotModel2017 = []() {
    RobotModel model;
    model.WheelRadius = 0.0281;
    // See doc/wheel_layout.txt for more information on wheel angles.
    model.WheelAngles = {
        DegreesToRadians(30), DegreesToRadians(360 - 39), DegreesToRadians(180 + 39),
        DegreesToRadians(180 - 30),
    };
    model.WheelDist = 0.0793;

    model.DutyCycleMultiplier = 9;  // TODO: tune this value

    model.recalculateBotToWheel();

    return model;
}();

const RobotModel RobotModelCurrent = []() {
    return RobotModel2017;
}();
