//
// Created by andrew on 11/25/21.
//

#include "Predictor.h"

Predictor::Predictor(bool debug) {
    this->debug = debug;
}

Point Predictor::predict(TargetAndCenter points, double t, double currentTime) {
    double radius = norm(points.targetCenter - points.buffCenter);
    Point relativeTarget = points.targetCenter - points.buffCenter;

    //current angle of the spoke of interest
    double currAngle = atan2(relativeTarget.y, relativeTarget.x);

    //there will be no time change if the callCount = 0 and will create a nan
    if(this->callCount > 0){
        double angleChange = currAngle - lastSpokeAngle;

        //phi diff is close to 2pi when it rolls over. might be other values so i added a while loop
        while(angleChange > 1)
            angleChange -= PI;

        double timeChange = currentTime - timeOfLastPredictCall;
        double currAngularVelocity =  - (angleChange / timeChange);

        if(debug)
            cout << "curr anglular velocity: " << currAngularVelocity << endl;

        //creates moving average where recent values are prioritized
        angVelocityAvg = (0.975) * angVelocityAvg + (0.025) * currAngularVelocity;
    }
    this->callCount++;

    //angle to predict into the future in radians
    double futureAngle = angVelocityAvg * t;

    //this delta_phi is used for how far we want to look into the future
    double angleToTravel = currAngle - futureAngle;

    if(debug)
        printf("average delta phi: %f \n", angVelocityAvg);

    lastSpokeAngle = currAngle;
    timeOfLastPredictCall = currentTime;

    Point target;
    target.x = radius * cos(angleToTravel) + points.buffCenter.x;
    target.y = radius * sin(angleToTravel) + points.buffCenter.y;
    return target;
}
