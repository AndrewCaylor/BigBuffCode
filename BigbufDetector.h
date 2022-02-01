
#ifndef BIGBUFF_BIGBUFDETECTOR_H
#define BIGBUFF_BIGBUFDETECTOR_H

//These file locations will change when integrated with rest of project
#include "util/common.h"  //RobotBase/vision
#include "util/message.h" //robogrinder SDK
#include "util/serial_port.h"
#include "Predictor.h"

/**
 * This class is for finding the targets of the big buff and predicting target location into the future.
 * Tests for this class can NOT be found in the main at https://github.com/robogrinder/CVB-Bigbuff
 *  WHO TF DELETED THE REPO???? I WAS STILL USING IT!!!!!!!!!!
 */
class BigbufDetector
{
public:
    BigbufDetector(serial_port serialPort, bool debug = false);

    //meant to be used publicly
    void feed_im(cv::Mat frame, OtherParam otherParam);

    double timeSinceEpoch();

    TargetAndCenter getTargetAndCenterPoints(cv::Mat frame, _color color);


    vector<Point> realSpokePoints = {};
    bool valid = false;

private:
    serial_port serialPort;

    Predictor *predictor;

    void outputToSerial(int pitch, int yaw);

    PitchAndYaw getPitchYaw(vector<Point> points);

    //if true, will print out messages to the console, and not use the serial port
    bool debug = false;

    //increments every cycle of predictFutureTargetLocation, used for timekeeping, especially when debugging
    int callCount = 0;

    /*
 * Vars and functions for finding the target / center
 */
    //Used for color thresholding
    //experimentally found values for doing target detection
    int Blue_minRed = 35, Blue_maxRed = 200;
    int Blue_minBlue = 220, Blue_maxBlue = 255;
    int Blue_minGreen = 220, Blue_maxGreen = 255;

    int Red_minRed = 20, Red_maxRed = 255;
    int Red_minBlue = 10, Red_maxBlue = 100;
    int Red_minGreen = 10, Red_maxGreen = 100;

    //Used for finding spoke with target
    //all of these "ratio" values are divided by 1000 before used
    int min_spoke_side_ratio = 0;
    int max_spoke_side_ratio = 480;
    int min_spoke_area_ratio = 0;
    int max_spoke_area_ratio = 430;

    //average side ratio: .62
    int min_target_side_ratio = 500;
    int max_target_side_ratio = 700;
    //average area ration : .85 (can be higher if very accurate rectangele)
    int min_target_area_ratio = 800;
    int max_target_area_ratio = 1000;

    //used for finding the bigbuff center
    //represents the distance to the center of the bigbuff in terms of the distance from the center of spoke to the center of the target
    int slopeCoeff = 3350;
    //distanceErrorCoeff * (distance from the center of spoke to the center of the target) = maxDistanceError
    int distanceErrorCoeff = 600;

    int element_size = 7;

    bool isSpoke(vector<Point> &contour);
    bool isTarget(vector<Point> &contour);
    Point getCenter(vector<Point> &contour);
    void display(Mat mask, const Mat &frame);

    //  These variables are for finding the real angles the motors need to turn from where the target was found in the image

    float x, y, z, width = 140.0f, height = 60.0f;
    int OFFSET_YAW = 3600;
    int OFFSET_PITCH = 3600;

    std::vector<cv::Point3f> real_armor_points;
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1293.5303221625442802, 0.3651215140945823, 355.9091806402759630,
                            0.0000000000000000, 1293.9256252855957428, 259.1868664367483461,
                            0.0000000000000000, 0.0000000000000000, 1.0000000000000000);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5)
                              << -0.2126367859619807,
                          0.2282910064864265, 0.0020583387355406, 0.0006136511397638, -0.7559987171745171);


// Contour for a spoke, used for comparison against other spokes, to find them bettere
    vector<Point> spokePoints = {
        Point(707, 409),
        Point(707, 410),
        Point(706, 411),
        Point(705, 411),
        Point(705, 413),
        Point(704, 414),
        Point(703, 414),
        Point(703, 416),
        Point(704, 416),
        Point(705, 417),
        Point(705, 418),
        Point(706, 418),
        Point(707, 419),
        Point(707, 420),
        Point(708, 420),
        Point(709, 421),
        Point(709, 422),
        Point(710, 422),
        Point(711, 423),
        Point(711, 424),
        Point(712, 424),
        Point(713, 425),
        Point(713, 426),
        Point(716, 426),
        Point(717, 427),
        Point(717, 428),
        Point(718, 429),
        Point(718, 430),
        Point(720, 430),
        Point(721, 431),
        Point(721, 432),
        Point(722, 432),
        Point(723, 433),
        Point(723, 434),
        Point(726, 434),
        Point(727, 435),
        Point(727, 436),
        Point(728, 436),
        Point(729, 437),
        Point(729, 438),
        Point(730, 438),
        Point(731, 439),
        Point(731, 440),
        Point(732, 440),
        Point(733, 441),
        Point(733, 442),
        Point(734, 442),
        Point(735, 443),
        Point(735, 444),
        Point(738, 444),
        Point(739, 445),
        Point(739, 446),
        Point(740, 446),
        Point(741, 447),
        Point(741, 448),
        Point(744, 448),
        Point(745, 449),
        Point(745, 452),
        Point(748, 452),
        Point(749, 453),
        Point(749, 454),
        Point(750, 454),
        Point(751, 455),
        Point(751, 456),
        Point(752, 456),
        Point(753, 457),
        Point(753, 458),
        Point(756, 458),
        Point(757, 459),
        Point(757, 466),
        Point(756, 467),
        Point(755, 467),
        Point(755, 468),
        Point(754, 469),
        Point(753, 469),
        Point(753, 470),
        Point(752, 471),
        Point(751, 471),
        Point(751, 474),
        Point(750, 475),
        Point(749, 475),
        Point(749, 480),
        Point(750, 480),
        Point(751, 481),
        Point(751, 482),
        Point(752, 482),
        Point(753, 483),
        Point(753, 484),
        Point(756, 484),
        Point(757, 485),
        Point(757, 486),
        Point(758, 486),
        Point(759, 487),
        Point(759, 488),
        Point(762, 488),
        Point(763, 489),
        Point(763, 490),
        Point(764, 490),
        Point(765, 491),
        Point(765, 492),
        Point(768, 492),
        Point(769, 493),
        Point(769, 494),
        Point(774, 494),
        Point(774, 493),
        Point(775, 492),
        Point(776, 492),
        Point(776, 491),
        Point(777, 490),
        Point(778, 490),
        Point(778, 489),
        Point(779, 488),
        Point(780, 488),
        Point(780, 487),
        Point(781, 486),
        Point(782, 486),
        Point(782, 485),
        Point(783, 484),
        Point(784, 484),
        Point(784, 481),
        Point(785, 480),
        Point(786, 480),
        Point(786, 479),
        Point(787, 478),
        Point(788, 478),
        Point(788, 477),
        Point(789, 476),
        Point(790, 476),
        Point(790, 475),
        Point(791, 474),
        Point(792, 474),
        Point(792, 473),
        Point(793, 472),
        Point(794, 472),
        Point(794, 469),
        Point(795, 468),
        Point(796, 468),
        Point(796, 467),
        Point(797, 466),
        Point(798, 466),
        Point(798, 465),
        Point(799, 464),
        Point(800, 464),
        Point(800, 459),
        Point(799, 459),
        Point(798, 458),
        Point(798, 457),
        Point(797, 457),
        Point(796, 456),
        Point(796, 455),
        Point(795, 455),
        Point(794, 454),
        Point(794, 453),
        Point(793, 453),
        Point(792, 452),
        Point(792, 451),
        Point(791, 451),
        Point(790, 450),
        Point(790, 449),
        Point(789, 449),
        Point(788, 448),
        Point(788, 447),
        Point(785, 447),
        Point(784, 446),
        Point(784, 445),
        Point(783, 445),
        Point(782, 444),
        Point(782, 443),
        Point(780, 443),
        Point(779, 442),
        Point(779, 441),
        Point(777, 441),
        Point(777, 442),
        Point(776, 443),
        Point(775, 443),
        Point(775, 444),
        Point(774, 445),
        Point(773, 445),
        Point(773, 446),
        Point(772, 447),
        Point(771, 447),
        Point(771, 448),
        Point(770, 449),
        Point(769, 449),
        Point(769, 452),
        Point(768, 453),
        Point(761, 453),
        Point(760, 452),
        Point(760, 451),
        Point(759, 451),
        Point(758, 450),
        Point(758, 449),
        Point(757, 449),
        Point(756, 448),
        Point(756, 447),
        Point(755, 447),
        Point(754, 446),
        Point(754, 445),
        Point(753, 445),
        Point(752, 444),
        Point(752, 443),
        Point(749, 443),
        Point(748, 442),
        Point(748, 441),
        Point(747, 441),
        Point(746, 440),
        Point(746, 439),
        Point(743, 439),
        Point(742, 438),
        Point(742, 435),
        Point(739, 435),
        Point(738, 434),
        Point(738, 431),
        Point(735, 431),
        Point(734, 430),
        Point(734, 429),
        Point(733, 429),
        Point(732, 428),
        Point(732, 427),
        Point(730, 427),
        Point(729, 426),
        Point(729, 425),
        Point(727, 425),
        Point(726, 424),
        Point(726, 423),
        Point(725, 423),
        Point(724, 422),
        Point(724, 421),
        Point(721, 421),
        Point(720, 420),
        Point(720, 417),
        Point(717, 417),
        Point(716, 416),
        Point(716, 415),
        Point(715, 415),
        Point(714, 414),
        Point(714, 413),
        Point(711, 413),
        Point(710, 412),
        Point(710, 409)
    };
};

#endif //BIGBUFF_BIGBUFDETECTOR_H
