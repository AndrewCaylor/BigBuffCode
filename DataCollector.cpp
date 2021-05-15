
#include <chrono>
#include <tgmath.h>
#include "DataCollector.h"
#include <cstdint>

using namespace cv;
using namespace std;

DataCollector::DataCollector(double radius, bool debug) {
    t_prev = timeSinceEpochMillisec();
    this->debug = debug;
    r = radius;
}

DataCollector::DataCollector() {}

Point DataCollector::predictConst(vector<Point> points, double t) {
    double xTarget = points[0].x - points[1].x;
    double yTarget = points[0].y - points[1].y;
    double phi_1 = (w_const * t) / r; //In radians
    double phi_2 = atan(yTarget / xTarget);
    double delta_phi = phi_2 - phi_1;
    Point target(0.0f, 0.0f);
    target.x = r * cos(delta_phi) + points[1].x;
    target.y = r * sin(delta_phi) + points[1].y;
    return target;
}

Point DataCollector::predictSine(vector<Point> points, double t) {
    double xTarget = points[0].x - points[1].x;
    double yTarget = points[0].y - points[1].y;
    double phi_1 = atan(prev_target.y / prev_target.x); // needs to be in radians
    if (prev_target.x < 0) { // Covers (-, -) condition and (-, +) condition
        phi_1 += 180;
    }
    else if (prev_target.y < 0) { // 4th quadrant needs to be positive
        phi_1 += 360;
    }
    double phi_2 = atan(yTarget / xTarget);
    if (xTarget < 0) { // Covers (-, -) condition and (-, +) condition
        phi_2 += 180;
    }
    else if (yTarget < 0) { // 4th quadrant needs to be positive
        phi_2 += 360;
    }
    double phi_3 = phi_1 - phi_2;
    double dt = ((double) (t_prev - timeSinceEpochMillisec())) / 1000;
    t_prev = timeSinceEpochMillisec();
    double w = (phi_3 * r) / dt; // Since this is just average w over interval, maybe we can find exact interval in the future
    double t1 = asin((w - C) / A) / B;
    double t2 = (PI / B) - t1;
    if (t1 < 0) {
        t1 += (PI / B);
    }
    double t_curr = 0;
    if (w_prev < w) { // This depends on what A, B, and C are but we know ahead of time
        t_curr = t1;
    }
    else {
        t_curr = t2;
    }
    w_prev = w;
    double t_pred = t_curr + t;
    double d_phi = -1 * (A / B) * cos(B * t_pred) + C * t_pred + (A / B) * cos(B * t_curr) - C * t_curr;
    double phi = phi_2 - d_phi; // Current angle minus the predicted change
    prev_target.x = r * cos(phi);
    prev_target.y = r * sin(phi);
    Point target(0.0f, 0.0f);
    target.x = r * cos(phi) + points[1].x;
    target.y = r * sin(phi) + points[1].y;
    return target;
}

uint64_t DataCollector::timeSinceEpochMillisec() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}
