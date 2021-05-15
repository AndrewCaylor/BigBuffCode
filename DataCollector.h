//
// Created by andrew on 2/20/21.
//

#ifndef BIGBUFF_DATACOLLECTOR_H
#define BIGBUFF_DATACOLLECTOR_H

#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;

class DataCollector {
private:

    //coefficients for the sine function
    double A = 0;
    double B = 0;
    double C = 0;

    // These are set beforehand
    double r = 0; //radius of the bigbuff
    double w_const = 0;
    double PI = 3.14159265;
    bool debug = false;

    // These are constantly changing
    uint64_t t_prev = 0;
    double w_prev = 0; // Assume these are stored in the data collector
    Point prev_target = Point(0.0f, 0.0f);
    uint64_t timeSinceEpochMillisec();
public:

    Point predictConst(vector<Point> points, double t);
    Point predictSine(vector<Point> points, double t);
    DataCollector(double radius, bool debug = false);
    DataCollector();

};


#endif //BIGBUFF_DATACOLLECTOR_H
