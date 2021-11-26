

//These file locations will change when integrated with rest of project
#include "util/serial_port.cpp"
#include <iostream>
#include "opencv2/opencv.hpp"
#include <chrono>
#include <thread>
#include <opencv2/core/core_c.h>

#include "BigbufDetector.h"

using namespace cv;
using namespace std;

#define B BigbufDetector

/**
 * modified version of feed_im made only for testing the the detection and prediction code
 * (does not use the serial port)
 */
void predictionUnitTest();

int main(){
    bool doUnitTest = true;

    if(doUnitTest){
        // runs the testing method that doe
        predictionUnitTest();
    }
    else{
        int framesRead = 0;
        Mat frame;

        VideoCapture cap;
        //test file
        const char *filePath = "/home/andrew/Desktop/capture1.mp4";

        cap.open(filePath);
        if (!cap.isOpened()) {
            cout << "Could not open video file.";
            return -1;
        }
        else{
            cout << "Video file opened.";
        }

        //initializing bigbuff detector with test variables
        OtherParam otherParam; //I did not make this struct just go with it
        otherParam.color = RED;
        otherParam.id = 0;
        otherParam.level = 0;
        otherParam.mode = BIGBUFF;
        serial_port serialPort;
        BigbufDetector bigbufDetector = BigbufDetector(serialPort, true);
        while (1) {
            cap.read(frame);
            if (frame.empty()) {
                cout << "Could not read frame " << framesRead;
                return -2;
            }

            bigbufDetector.feed_im(frame, otherParam);
            framesRead++;
            //comment this line out to use space to step through the video frames
//        waitKey(0);
        }
    }
}

void predictionUnitTest(){

    //set to true if you want to see gui
    bool showFrames = true;
    double secondsInFutureToPredict = .5;

    int framesRead = 0;
    Mat frame;

    VideoCapture cap;

    // path to test video file (Yes, you need the ../ )
    const char *filePath = "../capture1.mp4";

    cap.open(filePath);
    if (!cap.isOpened()) {
        cout << "Could not open video file.";
        return;
    }
    else{
        cout << "Video file opened.";
    }

    int totalFrames = cap.get(CAP_PROP_FRAME_COUNT);

    Point predictions[totalFrames];
    Point actual[totalFrames];
    bool fails[totalFrames];

    double framesPerSecond = cap.get(CAP_PROP_FPS);

    //number of frames between current location, and the frames where it is predicted at
    int offset = (int) (framesPerSecond * secondsInFutureToPredict);

    serial_port serialPort;
    BigbufDetector bigbufDetector = BigbufDetector(serialPort, showFrames);
    Predictor predictor = Predictor(true);

    int failCount = 0;
    double totalError = 0;
    for (int i = 0; i < totalFrames; i++) {

        cap.read(frame);
        if (frame.empty()) {
            cout << "Could not read frame " << framesRead;
            return;
        }

        TargetAndCenter targetAndCenter = bigbufDetector.getTargetAndCenterPoints(frame, RED);

        if(!targetAndCenter.failed){
            actual[i] = targetAndCenter.targetCenter;

            std::cout << "time since start: " << i * .033 << "seconds" << endl;

            Point futureEstimate = predictor.predict(targetAndCenter, secondsInFutureToPredict, .033 * i);
            predictions[i] = futureEstimate;
            fails[i] = false;
        }
        else{
            failCount ++;
            fails[i] = true;
        }

        if(i > offset){
            //if there were errors detecting at i or i - offset, then it can't display properly
            if(!fails[i] && !fails[i - offset]){
                double error =  norm(actual[i] - predictions[i - offset]);
                totalError += error;

                printf("prediction error: %lf pixels \n", error);

                //current value (red)
                circle(frame, actual[i], 10, Scalar(0,0,255), -1);
                //future prediction (green)
                circle(frame, predictions[i - offset], 10, Scalar(0,255,0), -1);
                //prediction generated on this frame (grey)
                circle(frame, predictions[i], 10, Scalar(100,100,100), -1);

                if(showFrames){
                    imshow("frame", frame);
                    waitKey(1);
                }
            }
            else{
                imshow("frame", frame);
            }

        }
    }
    printf("\ntotal average error: %lf pixels\n", totalError / (totalFrames - offset - failCount));
    printf("failcount: %d, totalFrames: %d\n", failCount, totalFrames);
}
