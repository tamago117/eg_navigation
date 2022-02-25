#pragma once
#include <string>

//addition status
enum class robot_status{
    run,//0
    stop,//1
    angleAdjust,//2
    safety_stop,//3
    recovery,//4
    column,//5
    speed_up,//6
    tracking//7
};

std::string robot_status_str(robot_status status)
{
    switch (status){
        case(robot_status::run):
            return "run";
        case(robot_status::stop):
            return "stop";
        case(robot_status::angleAdjust):
            return "angleAdjust";
        case(robot_status::safety_stop):
            return "safety_stop";
        case(robot_status::recovery):
            return "recovery";
        case(robot_status::column):
            return "column";
        case(robot_status::speed_up):
            return "speed_up";
        case(robot_status::tracking):
            return "recovery";
        default:
            return "";
    }
}