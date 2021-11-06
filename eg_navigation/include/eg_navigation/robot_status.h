#pragma once
#include <string>

//addition status
enum class robot_status{
    run,
    stop,
    angleAdjust,
    safety_stop,
    recovery
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
        default:
            return "";
    }
}