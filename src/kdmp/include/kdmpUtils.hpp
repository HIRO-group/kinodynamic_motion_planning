#ifndef KDMP_UTILS_
#define KDMP_UTILS_

#include <vector>
#include <string>
#include <sstream>
#include <iostream>

static inline void printVec(std::vector<double> vec, std::string label = "")
{
    std::stringstream ss;
    ss << label;
    ss<<"[ ";
    for (const auto &d : vec) {
        ss << " ";
        ss << d;
    }
    ss << "]";
    std::cout << ss.str() << std::endl;
}

static inline std::string vecToString(std::vector<double> vec) 
{
    std::stringstream ss;
    ss<<"[ ";
    for (const auto &d : vec) {
        ss << " ";
        ss << d;
    }
    ss << "]";
    return ss.str();
}
#endif