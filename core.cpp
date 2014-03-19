#include "core.h"
using namespace cv;
using namespace std;
Core::Core():
    isSetImageSizeDone(false),
    calibrationSamplesCounter(0),
    goodSamplesCounter(0),
    isCalibrationDone(false),
    minimumSamplesForCalibration(4),
    minimumPatternSizeWidth(2),
    minimumPatternSizeHeight(2),
    squareSize(0.1),
    isSetPatternSizeDone(false),
    pose(Matrix::eye(4)),
    viso(NULL),
    errorCode(0)
{
}
bool Core::addSampleToCalibration(Mat &calibrationImage){

    if(!isSetPatternSizeDone){
        cerr<<"SetPatternSize is not done (in Core::addSampleToCalibration())";
        return false;
    }
    if(patternSize.width<minimumPatternSizeWidth ||
       patternSize.height<minimumPatternSizeHeight){
        cerr<<"PatternSize is lower that required minimum (in Core::addSampleToCalibration())";
        return false;
    }
    if(!isSetImageSizeDone){
        cerr<<"Calibration image size is not set ( in Core::addSampleToCalibration())";
        return false;
    }
    if(calibrationImage.cols != imageSize.width || calibrationImage.rows!=imageSize.height){
        cerr<<"Calibration image size is not equal to internal imageSize ( in Core::addSampleToCalibration)";
        return false;
    }
    cv::vector<Point2f> corners;

    bool result;

    result = findChessboardCorners(calibrationImage,
                                   patternSize,
                                   corners,
                                   CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE );

    //We accept only samples with full visibility of corners
    if(result)
        goodSamplesCounter++;
    else{
        cerr<<"Didn't find chessboard corners' ( in Core::addSampleToCalibration())";
        return false;
    }

    //If we found corners then we need to do some subpix corners interpolation

    cornerSubPix(calibrationImage, corners, Size(11, 11), Size(-1, -1),
                 TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    imagePoints.push_back(corners);

    return true;
}
bool Core::calibrateCamera(std::string outputURL){

    if(goodSamplesCounter<minimumSamplesForCalibration){
        cerr<<"Not enough calibration samples (in Core::calibrateCamera()) "<<goodSamplesCounter;
        return false;
    }

    objectPoints.clear();
    //Assign values to objectPoints in object coordinate space
    for(int i = 0; i <goodSamplesCounter ; i++ ){
        objectPoints.push_back(vector<Point3f>());
        for(int j = 0; j < patternSize.height; j++ )
            for(int k = 0; k < patternSize.width; k++ )
                objectPoints[i].push_back(Point3f(float(j*squareSize), float(k*squareSize), 0));

    }

    vector<Mat> rvec, tvec;

    isCalibrationDone = cv::calibrateCamera(objectPoints,imagePoints,imageSize,cameraMatrix,distCoeffs,rvec,tvec);

    if(isCalibrationDone){
        saveCalibration(outputURL);
    }
    else{
        cerr<<"Calibration unsucessfull in Core::calibrateCamera()"<<std::endl;
        return false;
    }

    return isCalibrationDone;
}
bool Core::calibrateFromImages(std::string inputImagesPathURL , std::string outputCalibrationDataURL, int numberOfSamples){

    //Kalibracja z pliku
    for(int i=1; i<numberOfSamples;i++)
    {
        cv::Mat image;
        char base_name[256];
        sprintf(base_name,"%06d.JPG",i);
        string actualImageURL  = inputImagesPathURL +"/" + base_name;

        image = cv::imread(actualImageURL,CV_LOAD_IMAGE_GRAYSCALE);
        if(! image.data ){
            cout << "Could not open or find the image for calibration in (Core::calibrateFromImages()) " << std::endl ;
            break;
        }
        addSampleToCalibration(image);
    }
    return calibrateCamera(outputCalibrationDataURL);
}

void Core::saveCalibration(std::string outputURL){

     cv::FileStorage outputFile(outputURL, FileStorage::WRITE);
     if(!outputFile.isOpened()){
         cout<< "Couldnt open file";
         return;
     }
     outputFile<<"cameraMatrix"<<cameraMatrix;
     outputFile.release();
}

bool Core::loadCalibration(std::string inputURL){

    cv::FileStorage inputFile(inputURL, FileStorage::READ);
    if(!inputFile.isOpened()){
        std::cerr<<"Cannot open calibrationData file in Core::loadCalibration()";
        return false;
    }
    inputFile["cameraMatrix"] >> cameraMatrix;
    isCalibrationDone = true;
    return true;
}

bool Core::addImgToOdometry(cv::Mat img, bool replace){

    if(viso == NULL)
        return false;

    if(!isCalibrationDone){
        cerr<<"To do Odometry you need to calibrate Camera (in Core::addImgToOdometry()";
        return false;
    }
    if(!isSetImageSizeDone){
        cerr<<"Image size is not set ( in Core::addImgToOdometry())";
        return false;
    }
    if(img.cols != imageSize.width || img.rows!=imageSize.height){
        cerr<<"Image size is not equal to internal imageSize ( in Core::addImgToOdometry())";
        return false;
    }

    // image dimensions
    int32_t width= img.cols;
    int32_t height= img.rows;

    // compute visual odometry
    int32_t dims[] = {width,height,width};

    if (viso->process(img.datastart, dims,replace)) {
        pose = pose* Matrix::inv(viso->getMotion());
      }

    else {
        cout << " ... failed!" << endl;
        return false;
        errorCode=viso->errorCode;
      }
  return true;
}

bool Core::setPatternSize(cv::Size x){
    if(x.height>=minimumPatternSizeHeight && x.width>=minimumPatternSizeWidth){
        patternSize =x;
        isSetPatternSizeDone=true;
        return true;
    }
    else{
        cerr<<"Pattern size too small (in Core::setPatternSize)";
        return false;
    }
}
bool Core::createVisualOdometryMonoObject(){
    //Create visualOdometryMono object
    if(isCalibrationDone){
        VisualOdometryMono::parameters param;

        param.calib.f  = cameraMatrix.at<double>(0); // focal length in pixels
        param.calib.cu = cameraMatrix.at<double>(2); // principal point (u-coordinate) in pixels
        param.calib.cv = cameraMatrix.at<double>(5); // principal point (v-coordinate) in pixels

        viso=new VisualOdometryMono(param);
        return true;
    }
    else{
        cerr<<"Calibration not done (Core::createVisualOdometryMonoObject())";
        return false;
    }
}
void Core::changeInternalParameters(double height,
                                    double pitch,
                                    int32_t ransac_iters,
                                    double inlier_threshold,
                                    double motion_threshold){
    viso->param.height=height;
    viso->param.pitch=pitch;
    viso->param.ransac_iters= ransac_iters;
    viso->param.inlier_threshold=inlier_threshold;
    viso->param.motion_threshold=motion_threshold;

}

void Core::resetPose(){
    pose = Matrix::eye(4);
}
