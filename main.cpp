

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

//experimentally found values
int min_red = 35, max_red = 200;
int min_blue = 220, max_blue = 255;
int min_green = 220, max_green = 255;
int max_spoke_area = 1050;
int min_spoke_area = 800;
int max_target_area = 2000;
int min_target_area = 0;
int element_size = 9;
int curr_contour_idx = 0;

bool isSpoke(vector<Point> &contour);
bool isTarget(vector<Point> &contour);
void predictionUnitTest();

//real main
int main(){
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

    bool doUnitTest = false;

    if(doUnitTest){
        predictionUnitTest();
    }
    else{
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

/**
 * modified version of main made for testing the predicted values to the actual values
 */
void predictionUnitTest(){

    //set to true if you want to see gui
    bool showFrames = true;
    double secondsInFutureToPredict = .5;

    int framesRead = 0;
    Mat frame;

    VideoCapture cap;
    //test file
    const char *filePath = "/home/andrew/Desktop/capture1.mp4";

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

    int failCount = 0;

    double totalError = 0;
    for (int i = 0; i < totalFrames; i++) {

        cap.read(frame);
        if (frame.empty()) {
            cout << "Could not read frame " << framesRead;
            return;
        }

        B::TargetAndCenter targetAndCenter = bigbufDetector.getTargetAndCenterPoints(frame, RED);

        if(!targetAndCenter.failed){
            actual[i] = targetAndCenter.targetCenter;

            std::cout << "time since epoch" << bigbufDetector.timeSinceEpoch() << endl;

            Point futureEstimate = bigbufDetector.predictFutureTargetLocation(targetAndCenter, secondsInFutureToPredict, bigbufDetector.timeSinceEpoch());
            predictions[i] = futureEstimate;
            fails[i] = false;
        }
        else{
            failCount ++;
            fails[i] = true;
        }

        if(i > offset){
            //if ther are no errors at indices
            if(!fails[i] && !fails[i - offset]){
                double error =  norm(actual[i] - predictions[i - offset]);
                totalError += error;

                printf("prediction error: %lf pixels \n", error);

                //future prediction
                circle(frame, predictions[i - offset], 10, Scalar(0,255,0), -1);
                //current value
                circle(frame, actual[i], 10, Scalar(0,0,255), -1);
                //prediction for this frame
                circle(frame, predictions[i], 10, Scalar(255,255,0), -1);

                if(showFrames){
                    imshow("frame", frame);
                    waitKey(1);
                }
            }
            else{
                imshow("frame", frame);
//                waitKey(0);
            }

        }

//        usleep(33000);
    }
    printf("total average error: %lf \n", totalError / (totalFrames - offset - failCount));
    printf("failcount: %d, totalFrames: %d", failCount, totalFrames);
}



int asdf() {
    std::cout << cv::getVersionString();

    //total frames read
    int frames_read = 0;
    //total frames where we found the boxes
    int frames_captured = 0;
    Mat frame;
    VideoCapture cap;
    //test file

    const char *filePath = "/home/andrew/CLionProjects/CVB-Bigbuff/IMG_0270.mp4";

    cap.open(filePath);

    if (!cap.isOpened()) {
        cout << "Could not open video file.";
        return -1;
    }

    while (true) {
        vector<Point> spoke;
        int spoke_num = -1;
        vector<Point> target;
        int target_num = -1;

        // Making a window to modify fields
        namedWindow("Fields",WINDOW_NORMAL);
        createTrackbar("red_min","Fields",&min_red,255);
        createTrackbar("red_max","Fields",&max_red,255);
        createTrackbar("blue_min","Fields",&min_blue,255);
        createTrackbar("blue_max","Fields",&max_blue,255);
        createTrackbar("green_min","Fields",&min_green,255);
        createTrackbar("green_max","Fields",&max_green,255);
        createTrackbar("min_spoke_area","Fields",&min_spoke_area,2000);
        createTrackbar("max_spoke_area","Fields",&max_spoke_area,2000);
        createTrackbar("min_target_area","Fields",&min_target_area,2000);
        createTrackbar("max_target_area","Fields",&max_target_area,2000);
        createTrackbar("element_size","Fields",&element_size,30);
        createTrackbar("curr_contour_idx","Fields",&curr_contour_idx,50);

        //reading the next frame
        cap.read(frame);
        if (frame.empty()) {
            cout << "Could not read frame " << frames_read;
            return -2;
        }
        frames_read++;

        /*
         * for experimentation to split the channels into 3 so we can see things easier
        vector<Mat> ch;
        split(frame,ch);
        Mat R_ch,G_ch,B_ch;
        R_ch = ch[2]; G_ch = ch[1]; B_ch=ch[0];
        threshold(R_ch, R_ch, max_red,255,THRESH_TRUNC);
        threshold(R_ch, R_ch, min_red, 255, THRESH_TOZERO);
        threshold(B_ch, B_ch, max_blue,255,THRESH_TRUNC);
        threshold(B_ch, B_ch, min_blue, 255, THRESH_TOZERO);
        threshold(G_ch, G_ch, max_green,255,THRESH_TRUNC);
        threshold(G_ch, G_ch, min_green, 255, THRESH_TOZERO);

        resize(R_ch, R_ch, Size(500, 500));
        resize(B_ch, B_ch, Size(500, 500));
        resize(G_ch, G_ch, Size(500, 500));

        imshow("red", R_ch);
        imshow("blue", B_ch);
        imshow("green", G_ch);
        */

        //does the in range stuff and puts the result into mask
        Mat mask;
        //creates bitwise array: 1 in areas where the rgb values are within mins and max
        inRange(frame, Scalar(min_blue,min_green,min_red),Scalar(max_blue,max_green,max_red), mask);

        //used for passing into the dilate and erode methods
        Mat element  = getStructuringElement(MORPH_RECT, Size(element_size, element_size), Point(-1, -1));

        //expands pixels
        dilate(mask, mask, element);
        //smooths the image
        erode(mask, mask, element);

        //put to 500 so we can see the image and it doesnt take up the whole screen
        resize(mask, mask, Size(500, 500));
        resize(frame, frame, Size(500, 500));
        imshow("mask", mask);

        vector<vector<cv::Point>> contours;
        vector<Vec4i> hierarchy;
        //hierarchy[0,1,2,3] = [Next, Previous, First_Child, Parent]
        //passes in the mask and assigns the contours found to contours variable
        findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        /*
        if (curr_contour_idx < contours.size()) {
            vector<Point> curr_contour = contours[curr_contour_idx];
            RotatedRect rect = minAreaRect(curr_contour);
            Point2f pts[4];
            rect.points(pts);
            for (int i = 0; i < 4; i++) {
                line(frame, pts[i], pts[(i + 1) % 4], Scalar(255, 0, 0), 3);
            }
        }

        resize(frame, frame, Size(500, 500));
        imshow("frame", frame);
        */

        //looping through the contours to find the spoke contour
        for (int i = 0; i < contours.size(); i++) {
            vector<Point> curr_contour = contours[i];
            //target will be a child of the spoke
            if (hierarchy[i][2] >= 0 && isSpoke(curr_contour)) {
                spoke = curr_contour;
                spoke_num = i;
                break;
            }
        }

        //make sure that there is a spoke
        if (spoke_num == -1) {
            imshow("frame", frame);
            if(waitKey(1)>=0)
                break;
            continue;
        }

        //loop through the children of the spoke
        //hierarchy[i][0] is the next child of the contour
        //hierarchy[spoke_num][2] is the first child (greatest i)
        for (int i = hierarchy[spoke_num][2]; i >= 0; i = hierarchy[i][0]) {
            vector<Point> curr_contour = contours[i];
            if (isTarget(curr_contour)) {
                target = curr_contour;
                target_num = i;
                break;
            }
        }

        //ensure that we actually found a target
        if (target_num == -1) {
            imshow("frame", frame);
            if(waitKey(1)>=0)
                break;
            continue;
        }


        vector<Point> curr_contour = contours[target_num];
        RotatedRect rect = minAreaRect(curr_contour);
        Point2f pts[4];
        rect.points(pts);
        for (int i = 0; i < 4; i++) {
            line(frame, pts[i], pts[(i + 1) % 4], Scalar(255, 0, 0), 3);
        }

        imshow("frame", frame);
        // Display target's min_rect center
        frames_captured++;

        if(waitKey(1)>=0)
            break;

        // Display frames_read, frames_captured, and percent captured
    }
    return 0;
}

//TODO: make these functions better
//1: find the rectangleiness of the contour
//2: maybe allow for detection of rectangles if they are at an angle
//3: use side ratios to help find the rectangles that are the right dimensions
bool isSpoke(vector<Point> &contour) {
    //check that the area is in a certian range
    float curr_area = contourArea(contour);
    return curr_area > min_spoke_area && curr_area < max_spoke_area;
}

bool isTarget(vector<Point> &contour) {
    //check that the area is in a certian range
    float curr_area = cv::contourArea(contour);
    return curr_area > min_target_area && curr_area < max_target_area;
}

