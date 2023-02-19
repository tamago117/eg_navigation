#pragma once

namespace eg
{

template<class T> T constrain(T num, double minVal, double maxVal)
{
    if(num > maxVal){
        num = maxVal;
    }
    if(num < minVal){
        num = minVal;
    }

    return num;
}

} // namespace eg