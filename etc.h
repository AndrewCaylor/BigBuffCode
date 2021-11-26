
#ifndef BIGBUFF_ETC_H
#define BIGBUFF_ETC_H
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

constexpr double PI = 3.14159265;

/**
 * This struct is used for returning target and center info from functions
 */
struct TargetAndCenter{
    vector<Point> targetRect;   // the rectangle representing the target
    Point targetCenter;         // center of the target
    Point buffCenter;           // center of the bigBuff
    bool failed;                // if the point detection failed
};

/**
 *  Struct to store the pitch
 */
struct PitchAndYaw{
    int pitch;
    int yaw;
};

#endif //BIGBUFF_ETC_H
