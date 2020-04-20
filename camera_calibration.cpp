// Adapted from: https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <cassert>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;

#include <fstream>

static void help()
{
    cout <<  "This is a camera calibration sample." << endl
         <<  "Usage: calibration configurationFile"  << endl
         <<  "Near the sample file you'll find the configuration file, which has detailed help of "
                 "how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
}
class Settings
{
public:
    Settings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING, CHESSBOARD};
    enum InputType {INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST};

    Mat nextImage()
    {
        Mat result;
        if( inputCapture.isOpened() )
        {
            Mat view0;
            inputCapture >> view0;
            view0.copyTo(result);
        }
        else if( atImageList < (int)imageList.size() )
            result = imread(imageList[atImageList++], 1);

        return result;
    }

    static bool readStringList( const string& filename, vector<string>& l )
    {
        l.clear();
        FileStorage fs(filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back((string)*it);
        return true;
    }

    static bool isListOfImages( const string& filename)
    {
        string s(filename);
        // Look for file extension
        if( s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos )
            return false;
        else
            return true;
    }
public:
    Size boardSize;            // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;              // The number of frames to use from the input for calibration
    float aspectRatio;         // The aspect ratio
    int delay;                 // In case of a video input
    bool calibZeroTangentDist; // Assume zero tangential distortion
    bool calibFixPrincipalPoint;// Fix the principal point at the center
    bool flipVertical;          // Flip the captured images around the horizontal axis
    string outputFileName;      // The name of the file where to write
    bool showUndistorsed;       // Show undistorted images after calibration
    string input;               // The input ->



    int cameraID;
    vector<string> imageList;
    int atImageList;
    VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;

private:
    string patternToUse;


};


enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints );

int main(int argc, char* argv[])
{
    help();
    Settings s;
    const string inputSettingsFile = argc > 1 ? argv[1] : "default.xml";
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    clock_t prevTimestamp = 0;
    const char ESC_KEY = 27;

    for(int i = 0;;++i) {
        Mat view;

        view = s.nextImage();

        //-----  If no more image, or got enough, then stop calibration and show result -------------
//        if (mode == CAPTURING && imagePoints.size() >= (unsigned) s.nrFrames) {
//            if (runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints)) {
//                mode = CALIBRATED;
//                break;
//            }
//            else
//                mode = DETECTION;
//        }
        if (view.empty())          // If no more images then run calibration, save and stop loop.
        {
            if (imagePoints.size() > 0)
                runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints);
            break;
        }


        imageSize = view.size();  // Format input image.

        vector<Point2f> pointBuf;


        bool found = findChessboardCorners(view, s.boardSize, pointBuf,
                              CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);


        if (found)                // If done with success,
        {
            // improve the found corners' coordinate accuracy for chessboard
            if (s.calibrationPattern == Settings::CHESSBOARD) {
                Mat viewGray;
                cvtColor(view, viewGray, COLOR_BGR2GRAY);
                //cornerSubPix( viewGray, pointBuf, Size(11,11),
                //Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
            }

            if (mode == CAPTURING &&  // For camera only take new samples after delay time
                (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay * 1e-3 * CLOCKS_PER_SEC)) {
                imagePoints.push_back(pointBuf);
                prevTimestamp = clock();
                //blinkOutput = s.inputCapture.isOpened();
            }

            // Draw the corners.
            drawChessboardCorners(view, s.boardSize, Mat(pointBuf), found);
        }
    }

        //----------------------------- Output Text ------------------------------------------------
//        string msg = (mode == CAPTURING) ? "100/100" :
//                     mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
//        int baseLine = 0;
//        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
//        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);
//
//        if( mode == CAPTURING )
//        {
//            if(s.showUndistorsed)
//                msg = format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
//            else
//                msg = format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
//        }
//
//        putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

//        if( blinkOutput )
//            bitwise_not(view, view);

        //------------------------- Video capture  output  undistorted ------------------------------
//        if( mode == CALIBRATED && s.showUndistorsed )
//        {
//            Mat temp = view.clone();
//            undistort(temp, view, cameraMatrix, distCoeffs);
//        }

//        //------------------------------ Show image and check for input commands -------------------
//        imshow("Image View", view);
//        char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);
//
//        if( key  == ESC_KEY )
//            break;
//
//        if( key == 'u' && mode == CALIBRATED )
//            s.showUndistorsed = !s.showUndistorsed;
//
//        if( s.inputCapture.isOpened() && key == 'g' )
//        {
//            mode = CAPTURING;
//            imagePoints.clear();
//        }
        return 0;
    }

static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());



    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);

        //Fix
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch(patternType)
    {
        case Settings::CHESSBOARD:
            for( int i = 0; i < boardSize.height; ++i )
                for( int j = 0; j < boardSize.width; ++j )
                    corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
            break;
        default:
            break;
    }
}

static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr)
{

    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = 1.0;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, s.flag|CALIB_FIX_K4|CALIB_FIX_K5);


    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                            rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return true;
}

// Print camera parameters to the output file
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr )
{
    double fovx, fovy, focalLength, aspectRatio;
    Point_<double> pricipalPoint;

    calibrationMatrixValues(cameraMatrix, imageSize,  0, 0, fovx, fovy,
                            focalLength, pricipalPoint, aspectRatio);

    cout << distCoeffs << endl;

    ofstream outfile ("parameters.txt");
    if (outfile.is_open())
    {
        outfile << fovx << endl;
        outfile << fovy << endl;
        outfile << focalLength << endl;
        outfile << pricipalPoint << endl;
        outfile << aspectRatio << endl;
        outfile << distCoeffs.at<double>(0, 1) << endl;
        outfile << distCoeffs.at<double>(1, 1) << endl;
        outfile << distCoeffs.at<double>(2, 1) << endl;
        outfile  << distCoeffs.at<double>(3, 1) << endl;
        outfile.close();
    }
}

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,vector<vector<Point2f> > imagePoints )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s,imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
                             reprojErrs, totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
         << ". avg re projection error = "  << totalAvgErr ;

    if( ok )
        saveCameraParams( s, imageSize, cameraMatrix, distCoeffs, rvecs ,tvecs, reprojErrs,
                          imagePoints, totalAvgErr);
    return ok;
}
