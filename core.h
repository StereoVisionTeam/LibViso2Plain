#ifndef CORE_H
#define CORE_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <fstream>
#include <iostream>
#include "libviso2/viso_mono.h"
#include "libviso2/viso_stereo.h"



#define ODOMETRY_ERROR_NONE 00
#define ODOMETRY_ERROR_


class Core
{
public:
    Core();
    Matrix pose;

    int errorCode;

//Calibration
    bool addSampleToCalibration(cv::Mat &calibrationImage);
    bool calibrateCamera(std::string outputURL);
    bool setPatternSize(cv::Size x);
    void setPatternSize(int x, int y){ patternSize = cv::Size(x,y);
                                       isSetPatternSizeDone=true;}
    void setImageSize(cv::Size x) { imageSize= x;
                                isSetImageSizeDone=true;}
    void setsquareSize(double x){ squareSize= x;}
    void saveCalibration(std::string outputURL);
    bool loadCalibration(std::string inputURL);

    //Every image in directory inputURL should be named as follows : 000001.JPG 000002.JPG ... 000010.JPG ...
    bool calibrateFromImages(std::string inputURL,std::string outputCalibrationDataURL, int numberOfSamples);

//Odometry
    bool addImgToOdometry(cv::Mat img, bool replace=false);
    bool createVisualOdometryMonoObject();


    std::ofstream fileWithOdometry;

//Parameters
    void changeInternalParameters(double height,           // camera height above ground (meters)
                                  double pitch,            // camera pitch (rad, negative=pointing down)
                                  int32_t ransac_iters = 2000,     // number of RANSAC iterations
                                  double inlier_threshold = 0.00001, // fundamental matrix inlier threshold
                                  double motion_threshold = 100.0); // directly return false on small motions
private:

//Calibration
    bool isSetImageSizeDone;
    int calibrationSamplesCounter;
    int goodSamplesCounter;
    bool isCalibrationDone;

    int minimumSamplesForCalibration; //default =8
    int minimumPatternSizeWidth; //default =2
    int minimumPatternSizeHeight; //default =2
    double squareSize; //size of squares in calibration chessboard;
    cv::vector<cv::vector<cv::Point3f> > objectPoints;
    cv::Size patternSize; //Number of inner corners per a chessboard row and column for (8x8) board it's (7x7)
    bool isSetPatternSizeDone;
    cv::Mat cameraMatrix, distCoeffs;
    cv::vector<cv::vector<cv::Point2f> > imagePoints;
//Odometry
    VisualOdometryMono *viso;

//Basic
    cv::Size imageSize;

    void resetPose();
};

#endif // CORE_H
