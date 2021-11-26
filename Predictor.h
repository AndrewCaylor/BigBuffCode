//
// Created by andrew on 11/25/21.
//

#ifndef BIGBUFF_PREDICTOR_H
#define BIGBUFF_PREDICTOR_H

#include "etc.h"

class Predictor {
public:

    /**
     * @param debug enabling debug prints out messages
     */
    Predictor(bool debug = false);

    /**
     * Predicts where the target will be t seconds into the future.
     * This function assumes it will be called 30 times a second
     *
     * @param points = [current target point, current center point]
     * @param t time in future to predict
     * @return Point where the target will be in t seconds
     */
    Point predict(TargetAndCenter points, double t, double currentTime);

private:
    //increments every cycle of predictFutureTargetLocation, used for timekeeping, especially when debugging
    int callCount = 0;
    double angVelocityAvg = 0;
    double lastSpokeAngle = 0;
    double timeOfLastPredictCall = 0;

    bool debug = false;
};


#endif //BIGBUFF_PREDICTOR_H
