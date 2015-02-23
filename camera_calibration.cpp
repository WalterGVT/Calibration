#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
//#include <chrono>
#define WIN64

#include "FlyCapture2.h"


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

//#define FROMCAMERA
//#define FROMAVIFILE2  // lettura preventiva di tutto l'AVI
//#define FROMAVIFILE  // lettura del frame nel ciclo di elaborazione
#define FROMIMAGES

#define CALIBRAZIONE
//#define TRACKING

using namespace cv;
using namespace std;

int cvCheckChessboard_W(IplImage* src, CvSize size); // aggiunto il prototipo della funzione che fa il check, vedi (*). Implementazione in checkchessboard.cpp nell'attuale project


//// the Model
//#define N 4
//CvPoint3D32f modelPoints[N] = {
//		{ 0.0f, 0.0f, 0.0f },
//		{ -L_RIGHT, -H_ARM, L_FRONT_ARM },
//		{ L_LEFT, -H_ARM, L_FRONT_ARM },
//		{ 0.0f, -L_ANTENNA - H_ARM, L_FRONT_ARM + D_ANTENNA }
//};
//
//
//// camera parameters
////double _intrinsics[9] =
////	{ 531.9163709104515192, 0., 299.9301060718951817, 0.,
////	530.9958838480581562, 245.9404069432019924, 0., 0., 1. };
//double _intrinsics[9] =
//{ 530, 0., W / 2,
//0., 530, H / 2,
//0., 0., 1. };
//int focalPoint = 530;
//CvMat intrinsics = cvMat(4, 4, CV_64F, _intrinsics);


//class Camera
//{
//	//cv::VideoCapture* cam;
//	cv::Mat_<float> intr, extr;
//public:
//	Camera(int n = 0)
//	{
//		intr = cv::Mat_<float>(3, 3, 0.f);
//		// parametri interni 
//		intr.at<float>(0, 0) = 2.7105506628580332e+02;
//		intr.at<float>(0, 1) = 0;
//		intr.at<float>(0, 2) = 1.5950000000000000e+02;
//
//		intr.at<float>(1, 0) = 0;
//		intr.at<float>(1, 1) = 2.7105506628580332e+02;
//		intr.at<float>(1, 2) = 1.1950000000000000e+02;
//
//		intr.at<float>(2, 0) = 0;
//		intr.at<float>(2, 1) = 0;
//		intr.at<float>(2, 2) = 1;
//
//		//cam = new cv::VideoCapture(n);
//		//if (!cam->isOpened())
//		//	std::cerr << "arg\n";
//
//
//	}
//
//	void calib_extrinsic()
//	{
//		cv::namedWindow("w");
//		std::vector<cv::Point3f> obj_points;
//		std::vector<cv::Point2f> img_points;
//		std::vector<cv::Point2f> corners;
//		cv::Size size(8, 6);
//		float cell_size = 1;
//
//		for (int i = 0; i < size.height; ++i)
//			for (int j = 0; j < size.width; ++j)
//				obj_points.push_back(cv::Point3f(float(j*cell_size),
//				float(i*cell_size), 0.f));
//
//		cv::Mat img, gray;
//
//		while (1) {
//			*cam >> img;
//			cv::cvtColor(img, gray, CV_BGR2GRAY);
//
//			img_points.clear();
//			corners.clear();
//			bool found = cv::findChessboardCorners(gray, size, corners,
//				CV_CALIB_CB_ADAPTIVE_THRESH
//				| CV_CALIB_CB_FILTER_QUADS);
//
//			if (found) {
//				// cv::cornerSubPix(gray, corners,
//				//                  cv::Size(11, 11),
//				//                  cv::Size(-1, -1),
//				//                  cv::TermCriteria(CV_TERMCRIT_EPS
//				//                                   | CV_TERMCRIT_ITER, 30, 0.1));
//				cv::drawChessboardCorners(img, size, cv::Mat(corners), found);
//			}
//
//			cv::imshow("w", img);
//			int key = cv::waitKey(15) & 0xff;
//			//std::cout << found << " " << key << "\n";
//			if (key == ' ' && found) {
//
//				cv::Mat_<float> distCoeffs(4, 1, 0.f);
//				cv::Mat_<float> r(3, 3, 0.f);
//				cv::Mat_<float> rvecs(3, 1, 0.f);
//				cv::Mat_<float> tvecs(3, 1, 0.f);
//
//				cv::solvePnP(cv::Mat(obj_points), cv::Mat(corners), intr, distCoeffs, rvecs, tvecs);
//				cv::Rodrigues(rvecs, r);
//				extr = cv::Mat_<float>(4, 4, 0.f);
//
//				for (int y = 0; y < 3; y++) {
//					for (int x = 0; x < 3; x++)
//						extr.at<float>(y, x) = r.at<float>(y, x);
//
//					extr.at<float>(y, 3) = tvecs.at<float>(y, 0);
//				}
//
//				extr.at<float>(3, 0) = 0.f;
//				extr.at<float>(3, 1) = 0.f;
//				extr.at<float>(3, 2) = 0.f;
//				extr.at<float>(3, 3) = 1.f;
//
//				for (int y = 0; y < 4; y++) {
//					for (int x = 0; x < 4; x++) {
//						printf("%+.4e ", extr.at<float>(y, x));
//					}
//					putchar('\n');
//				}
//				putchar('\n');
//
//				//break;
//			}
//		}
//
//
//	}
//};

vector<string> imagess;


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
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType {INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST};

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{" << "BoardSize_Width"  << boardSize.width
                  << "BoardSize_Height" << boardSize.height
                  << "Square_Size"         << squareSize
                  << "Calibrate_Pattern" << patternToUse
                  << "Calibrate_NrOfFrameToUse" << nrFrames
                  << "Calibrate_FixAspectRatio" << aspectRatio
                  << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
                  << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

                  << "Write_DetectedFeaturePoints" << bwritePoints
                  << "Write_extrinsicParameters"   << bwriteExtrinsics
                  << "Write_outputFileName"  << outputFileName

                  << "Show_UndistortedImage" << showUndistorsed

                  << "Input_FlipAroundHorizontalAxis" << flipVertical
                  << "Input_Delay" << delay
                  << "Input" << input
           << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Square_Size"]  >> squareSize;
        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Write_DetectedFeaturePoints"] >> bwritePoints;
        node["Write_extrinsicParameters"] >> bwriteExtrinsics;
        node["Write_outputFileName"] >> outputFileName;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Show_UndistortedImage"] >> showUndistorsed;
        node["Input"] >> input;
        node["Input_Delay"] >> delay;
        interprate();
    }
    void interprate()
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        if (nrFrames <= 0)
        {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }

        if (input.empty())      // Check for valid input
                inputType = INVALID;
        else
        {
            if (input[0] >= '0' && input[0] <= '9')
            {
                stringstream ss(input);
                ss >> cameraID;
                inputType = CAMERA;
            }
            else
            {
                if (readStringList(input, imageList))
                    {
                        inputType = IMAGE_LIST;
                        nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
                    }
                else
                    inputType = VIDEO_FILE;
            }
            if (inputType == CAMERA)
                inputCapture.open(cameraID);
            if (inputType == VIDEO_FILE)
                inputCapture.open(input);
            if (inputType != IMAGE_LIST && !inputCapture.isOpened())
                    inputType = INVALID;
        }
        if (inputType == INVALID)
        {
            cerr << " Inexistent input: " << input;
            goodInput = false;
        }

        flag = 0;
        if(calibFixPrincipalPoint) flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
        if(calibZeroTangentDist)   flag |= CV_CALIB_ZERO_TANGENT_DIST;
        if(aspectRatio)            flag |= CV_CALIB_FIX_ASPECT_RATIO;


        calibrationPattern = NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
        if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
        if (calibrationPattern == NOT_EXISTING)
            {
                cerr << " Inexistent camera calibration mode: " << patternToUse << endl;
                goodInput = false;
            }
        atImageList = 0;

    }
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
            result = imread(imageList[atImageList++], CV_LOAD_IMAGE_COLOR);

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
public:
    Size boardSize;            // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;              // The number of frames to use from the input for calibration
    float aspectRatio;         // The aspect ratio
    int delay;                 // In case of a video input
    bool bwritePoints;         //  Write detected feature points
    bool bwriteExtrinsics;     // Write extrinsic parameters
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


static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints );

#ifdef CALIBRAZIONE
// ----------------------------------------------------------------------------
// Programma di calibrazione della telecamera.
// ----------------------------------------------------------------------------
//  input  : nome file in_CAL_PG.xml. Contiene le info geometriche della scacchiera
//  output file :  out_camera_data_PG.xml (nome definito in in_CAL_PG.xml)
//
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
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

    for(int i = 0;;++i)
    {
      Mat view;
      bool blinkOutput = false;

      view = s.nextImage();

      //-----  If no more image, or got enough, then stop calibration and show result -------------
      if( mode == CAPTURING && imagePoints.size() >= (unsigned)s.nrFrames )
      {
          if( runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints))
              mode = CALIBRATED;
          else
              mode = DETECTION;
      }
      if(view.empty())          // If no more images then run calibration, save and stop loop.
      {
            if( imagePoints.size() > 0 )
                runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints);
            break;
      }


        imageSize = view.size();  // Format input image.
        if( s.flipVertical )    flip( view, view, 0 );

        vector<Point2f> pointBuf;

        bool found;
        switch( s.calibrationPattern ) // Find feature points on the input format
        {
        case Settings::CHESSBOARD:
            found = findChessboardCorners( view, s.boardSize, pointBuf,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf );
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
            break;
        default:
            found = false;
            break;
        }

        if ( found)                // If done with success,
        {
              // improve the found corners' coordinate accuracy for chessboard
                if( s.calibrationPattern == Settings::CHESSBOARD)
                {
                    Mat viewGray;
                    cvtColor(view, viewGray, COLOR_BGR2GRAY);
                    cornerSubPix( viewGray, pointBuf, Size(11,11),
                        Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
                }

                if( mode == CAPTURING &&  // For camera only take new samples after delay time
                    (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) )
                {
                    imagePoints.push_back(pointBuf);
                    prevTimestamp = clock();
                    blinkOutput = s.inputCapture.isOpened();
                }

                // Draw the corners.
                drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );
        }

        //----------------------------- Output Text ------------------------------------------------
        string msg = (mode == CAPTURING) ? "100/100" :
                      mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(s.showUndistorsed)
                msg = format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
        }

        putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

        if( blinkOutput )
            bitwise_not(view, view);

        //------------------------- Video capture  output  undistorted ------------------------------
        if( mode == CALIBRATED && s.showUndistorsed )
        {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }

        //------------------------------ Show image and check for input commands -------------------
        imshow("Image View", view);
        char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

        if( key  == ESC_KEY )
            break;

        if( key == 'u' && mode == CALIBRATED )
           s.showUndistorsed = !s.showUndistorsed;

        if( s.inputCapture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }
    }

    // -----------------------Show the undistorted image for the image list ------------------------
    if( s.inputType == Settings::IMAGE_LIST && s.showUndistorsed )
    {
        Mat view, rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
            imageSize, CV_16SC2, map1, map2);

        for(int i = 0; i < (int)s.imageList.size(); i++ )
        {
            view = imread(s.imageList[i], 1);
            if(view.empty())
                continue;
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View", rview);
            char c = (char)waitKey();
            if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                break;
        }
    }


    return 0;
}
#endif

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
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

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
    case Settings::CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
        break;

    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
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
    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = 1.0;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, s.flag|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                             rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

// Print camera parameters to the output file
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr )
{
    FileStorage fs( s.outputFileName, FileStorage::WRITE );

    time_t tm;
    time( &tm );
    //struct tm *t2 = localtime( &tm );
	struct tm t2; 
	localtime_s(&t2, &tm);
	char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", &t2 );

    fs << "calibration_Time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_Width" << imageSize.width;
    fs << "image_Height" << imageSize.height;
    fs << "board_Width" << s.boardSize.width;
    fs << "board_Height" << s.boardSize.height;
    fs << "square_Size" << s.squareSize;

    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "FixAspectRatio" << s.aspectRatio;

    if( s.flag )
    {
        sprintf_s( buf, "flags: %s%s%s%s",
            s.flag & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
            s.flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
            s.flag & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
            s.flag & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );

    }

    fs << "flagValue" << s.flag;

    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;

    fs << "Avg_Reprojection_Error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "Extrinsic_Parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "Image_points" << imagePtMat;
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

// mezza schifezza, ma facciamo cosi per far prima
static void LoadImagesFromJpg(){
	imagess.push_back("I1.jpg");
	imagess.push_back("I2.jpg");
	imagess.push_back("I3.jpg");
	imagess.push_back("I4.jpg");
	imagess.push_back("I5.jpg");
	imagess.push_back("I6.jpg");
	imagess.push_back("I7.jpg");
	imagess.push_back("I8.jpg");
	imagess.push_back("I9.jpg");
	imagess.push_back("I10.jpg");
	imagess.push_back("I11.jpg");
	imagess.push_back("I12.jpg");
	imagess.push_back("I13.jpg");
	imagess.push_back("I14.jpg");
	imagess.push_back("I15.jpg");
	imagess.push_back("I16.jpg");
	imagess.push_back("I17.jpg");
	imagess.push_back("I18.jpg");
	imagess.push_back("I19.jpg");
	imagess.push_back("I20.jpg");
	imagess.push_back("I21.jpg");
	imagess.push_back("I22.jpg");
	imagess.push_back("I23.jpg");
	imagess.push_back("I24.jpg");
	imagess.push_back("I25.jpg");
	imagess.push_back("I26.jpg");
	imagess.push_back("I27.jpg");
	imagess.push_back("I28.jpg");
	imagess.push_back("I29.jpg");
	imagess.push_back("I30.jpg");
}

// naltra mezza schifezza
static void LoadImagesFromPng(){
	imagess.push_back("vlcsnap-1.png");
	imagess.push_back("vlcsnap-2.png");
	imagess.push_back("vlcsnap-3.png");
	imagess.push_back("vlcsnap-4.png");
}





using namespace FlyCapture2;

#ifdef TRACKING
// ---------------------------------------------------
// Programma per il test delle procedure di tracking
// ---------------------------------------------------
int main(int argc, char* argv[])
{
	Error error;
	Camera camera;
	CameraInfo camInfo;

	Mat intr(3, 3, DataType<double>::type, 0.f);;
	Mat extr;


	// parametri interni telecamera FLEA PTG
	intr.at<double>(0, 0) = 2.8639100807353752e+03;
	intr.at<double>(0, 1) = 0;
	intr.at<double>(0, 2) = 6.3950000000000000e+02;

	intr.at<double>(1, 0) = 0;
	intr.at<double>(1, 1) = 2.8639100807353752e+03;
	intr.at<double>(1, 2) = 5.1150000000000000e+02;

	intr.at<double>(2, 0) = 0;
	intr.at<double>(2, 1) = 0;
	intr.at<double>(2, 2) = 1;

	// coefficenti distorsione
	//				Mat distCoeffs(5, 1, 0.f);
	//				distCoeffs.at<double>(0, 0) = -7.9429030485311095e-02;
	//				distCoeffs.at<double>(0, 0) = -7.9429030485311095;
	Mat distCoeffs(5, 1, DataType<double>::type, 0.f);
	distCoeffs.at<double>(0) = 0.f;
	distCoeffs.at<double>(1) = -1.5359767720433414e+01;
	distCoeffs.at<double>(2) = 0.;
	distCoeffs.at<double>(3) = 0.;
	distCoeffs.at<double>(4) = 7.9923762518721446e+02;



	// coordinate modello
	vector<Point3f> obj_points;
	Size size(8, 6);
	float cell_size = 24; // 24 mm 

	for (int i = 0; i < size.height; ++i)
		for (int j = 0; j < size.width; ++j)
			obj_points.push_back(cv::Point3f(float(j*cell_size),
			float(i*cell_size), 0.f));

	//vector<Point2f> img_points;


#ifdef FROMAVIFILE2
//	//------------------------------------------------------------
	// Prepara array di immagini leggendo un file AVI
	//------------------------------------------------------------
	VideoCapture MyCapture;
	vector<Mat> vecImages(5200);
	Mat iFrame;
//	bool isOpen =  MyCapture.open("C:\\Users\\Walter Vanzella\\Documents\\Visual Studio 2013\\Projects\\SaveAVI\\SaveAVI\\fc2_save_2014-08-26-103306-0000.avi");
	bool isOpen = MyCapture.open("G:\\Head Tracking\\checkboard.avi");
	bool uscita = true;
	int ii = 0;
	while (uscita) {
		uscita = MyCapture.grab();
		uscita = MyCapture.retrieve(iFrame);
		if (uscita) {
			vecImages[ii] = Mat();
			iFrame.copyTo(vecImages[ii]);
			ii++;
		}
	}
	int nFrames = ii;
	// Visualizza
	//for (int j = 0; j < nFrames; j++){
	//	imshow("image", vecImages[j]);
	//	waitKey(30);
	//}

	//destroyAllWindows();
#endif




#ifdef FROMCAMERA
	//-------------------------------------------------------
	// Connect the camera
	//-------------------------------------------------------
	error = camera.Connect(0);
	if (error != PGRERROR_OK)
	{
		std::cout << "Failed to connect to the camera" << std::endl;
		return false;
	}

	//error = camera.SetProperty()

	// Get the camera info and print it out
	error = camera.GetCameraInfo(&camInfo);
	if (error != PGRERROR_OK)
	{
		std::cout << "Failed to get camera info from camera" << std::endl;
		return false;
	}
	std::cout << camInfo.vendorName << " "
		<< camInfo.modelName << " "
		<< camInfo.serialNumber << std::endl;

	error = camera.StartCapture();
	if (error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED)
	{
		std::cout << "Bandwidth exceeded" << std::endl;
		return false;
	}
	else if (error != PGRERROR_OK)
	{
		std::cout << "Failed to start image capture" << std::endl;
		return false;
	}
	// -------------------------------- End connect camera

#endif

#ifdef FROMIMAGES
	//------------------------------------------------------------
	// Prepara array di nomi immagini che poi leggera
	//------------------------------------------------------------
//	LoadImagesFromJpg();
	LoadImagesFromPng();
	//vecImages.resize(k_numImages);
#endif




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

	// capture loop
	Mat view;
	char key = 0;
	bool flagPlot=true;
	//i = 0;
	//auto start1 = chrono::system_clock::now();
	clock_t start1 = clock();
	int frames = 0;

#ifdef FROMAVIFILE2
	frames = 2400;
	while ((key != 'q') && (frames <nFrames)) // per  AVI
	{
		// -------------------------------------------------
		// Lettura delle immagini gia caricate da file AVI
		// -------------------------------------------------
		view = vecImages[frames++];

#endif


#ifdef FROMAVIFILE
		int da = 2400;
		int a = 5000;
		bool uscita;
		VideoCapture MyCapture;
		bool isOpen = MyCapture.open("G:\\Head Tracking\\checkboard.avi");
		if (isOpen == false) {
			cout << "File not found. " << endl;
			return -1;
		}

		while (key != 'q')  
		{

			uscita = MyCapture.grab();
			uscita = MyCapture.retrieve(view);
			if (uscita)
				frames++;
			else
				break;

			if ((frames < da) || (frames > a))
				continue;

#endif


#ifdef FROMCAMERA
	while (key != 'q')
	{
		// --------------------------------------
		// Acquisizione da telecamera
		// --------------------------------------
		// Get the image
		Image rawImage;
		Error error = camera.RetrieveBuffer(&rawImage);

		//auto start2 = chrono::system_clock::now();
		clock_t start2 = clock();
		double elapsed_secs = double(start2 - start1) / CLOCKS_PER_SEC;
		double actualFrRate = 1 / elapsed_secs;
		start1 = start2;
		frames++;

		if (key == 'w')
			if (flagPlot) 
				flagPlot=false;
			else 
				flagPlot = true;

		//if (flagPlot)
		//	cout << "frame " << frames << " NOT found.   Fps = " << actualFrRate << endl;


		if (error != PGRERROR_OK)
		{
			std::cout << "capture error" << std::endl;
			continue;
		}

		// convert to rgb
		Image rgbImage;
		rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);

		// convert to OpenCV Mat
		unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize() / (double)rgbImage.GetRows();
		Mat view = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);
#endif


#ifdef FROMIMAGES
	while (key != 'q')
	{

//		// -------------------------------------------------
//		// Lettura di una immagine salvata presa a caso
//		// -------------------------------------------------
//		string path = "C:\\Users\\Walter Vanzella\\Documents\\Visual Studio 2013\\Projects\\Project1_calibration\\x64\\Debug\\images\\";
		string path = "G:\\Head Tracking\\";
		int random = rand() % 4;
//		int random = rand() % 29;
		string nomefile = path + imagess.at(random);
//		view = imread("C:\\Users\\Walter Vanzella\\Documents\\Visual Studio 2013\\Projects\\Project1_calibration\\x64\\Debug\\images\\I17.jpg");
		view = imread(nomefile);
#endif




		clock_t begin = clock();
//		resize(view, view, Size(), 0.5, 0.5, INTER_NEAREST);
		clock_t end = clock(); double elapsed_secs0 = double(end - begin) / CLOCKS_PER_SEC;

		vector<Point2f> pointBuf;
		bool found;
		// sovrascrivo le dimensioni che sono state lette 
		s.boardSize.height = 6;
		s.boardSize.width = 8;
		Mat viewGray;
		cvtColor(view, viewGray, CV_BGR2GRAY);
		IplImage iplimg = viewGray; // conversione Mat to IplImage (smart)
		bool check_chessboard_result;

		begin = clock();
		switch (s.calibrationPattern) // Find feature points on the input format
		{
		case Settings::CHESSBOARD:
			//
			// Controllo della presenza della scacchiera. Inserito nel progetto con sostituzione nomi (_W  per il resto identico alla versione opencv)
			// allo scopo di studiarlo in debug.
			//
			//check_chessboard_result = cvCheckChessboard_W(&iplimg, s.boardSize);
			//check_chessboard_result = cvCheckChessboard(&iplimg, s.boardSize);
			//if (check_chessboard_result == true){
			//	int stop = 1;
			//}

//			found = findChessboardCorners(view, s.boardSize, pointBuf,CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE); // veloce
			found = findChessboardCorners(view, s.boardSize, pointBuf,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK ); // lento
//			found = findChessboardCorners( view, s.boardSize, pointBuf,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

			break;
		case Settings::CIRCLES_GRID:
		    found = findCirclesGrid( view, s.boardSize, pointBuf );
		    break;
		case Settings::ASYMMETRIC_CIRCLES_GRID:
		    found = findCirclesGrid( view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
		    break;
		default:
		    found = false;
		    break;
		}
		end = clock();double elapsed_secs1 = double(end - begin) / CLOCKS_PER_SEC;

		//found = false;
		if ( found)               
		{


		        // migliora corners' accuratezza delle coordinate 
				begin = clock();
		        if( s.calibrationPattern == Settings::CHESSBOARD)
		        {
		            Mat viewGray;
		            cvtColor(view, viewGray, COLOR_BGR2GRAY);
					cornerSubPix(viewGray, pointBuf, Size(11, 11),
		                Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
		        }
				end = clock(); double elapsed_secs2 = double(end - begin) / CLOCKS_PER_SEC;

		        //if( mode == CAPTURING &&  // For camera only take new samples after delay time
		        //    (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) )
		        //{
		        //    imagePoints.push_back(pointBuf);
		        //    prevTimestamp = clock();
		        //    blinkOutput = s.inputCapture.isOpened();
		        //}
		

		        // Draw the corners.
				double elapsed_secs3 = 0.;
				if (flagPlot) {
					begin = clock();
					drawChessboardCorners(view, s.boardSize, Mat(pointBuf), found);
					end = clock();
					elapsed_secs3 = double(end - begin) / CLOCKS_PER_SEC;
				}

				//
				// Calcola la posa 3D
				//
				//Mat_<float> rvecs(3, 1, 0.f);
				Mat r(3, 3, 0.f);
				Mat rvecs(3, 1, DataType<double>::type, 0.f);
				Mat tvecs(3, 1, DataType<double>::type, 0.f);

//				solvePnP(cv::Mat(obj_points), cv::Mat(pointBuf), intr, distCoeffs, rvecs, tvecs);
				begin = clock();
				solvePnP(obj_points, pointBuf, intr, distCoeffs, rvecs, tvecs);
				end = clock();
				double elapsed_secs4 = double(end - begin) / CLOCKS_PER_SEC;



				begin = clock();
				Rodrigues(rvecs, r);
				extr = Mat(4, 4, DataType<double>::type, 0.f);

				for (int y = 0; y < 3; y++) {
					for (int x = 0; x < 3; x++)
						extr.at<double>(y,x) = r.at<double>(y, x);

					extr.at<double>(y, 3) = tvecs.at<double>(y, 0);
				}

				extr.at<double>(3, 0) = 0.f;
				extr.at<double>(3, 1) = 0.f;
				extr.at<double>(3, 2) = 0.f;
				extr.at<double>(3, 3) = 1.f;

				////
				//// Visualizzazione parametri telecamera, rvecs e tvecs
				////
				//for (int y = 0; y < 4; y++) {
				//	for (int x = 0; x < 4; x++) {
				//		printf("%+.4e ", extr.at<double>(y, x));
				//	}
				//	putchar('\n');
				//}
				//putchar('\n');

				//std::cout << "rvecs: " << rvecs << std::endl;
				//std::cout << "tvecs: " << tvecs << std::endl;


				//
				// Riproiezione dei punti 
				//
				//std::vector<cv::Point2f> projectedPoints;
				//cv::projectPoints(obj_points, rvecs, tvecs, intr, distCoeffs, projectedPoints);
				//for (unsigned int i = 0; i < projectedPoints.size(); ++i)
				//{
				//	std::cout << "Image point: " << pointBuf[i] << " Projected to " << projectedPoints[i] << std::endl;
				//}

				// plot sistema di riferimento
				double elapsed_secs5 = 0.;
				if (flagPlot) {
					vector<Point3f> SdR;
					float lungVersore = 40.; // mm
					SdR.push_back(cv::Point3f(0.f, 0.f, 0.f));
					SdR.push_back(cv::Point3f(lungVersore, 0.f, 0.f));
					SdR.push_back(cv::Point3f(0.f, lungVersore, 0.f));
					SdR.push_back(cv::Point3f(0.f, 0.f, -lungVersore)); // occhio Z asse principale della tc 
					std::vector<cv::Point2f> ppSdR;
					cv::projectPoints(SdR, rvecs, tvecs, intr, distCoeffs, ppSdR);

					line(view, ppSdR.at(0), ppSdR.at(1), Scalar(255, 0, 0), 2, 8, 0);
					line(view, ppSdR.at(0), ppSdR.at(2), Scalar(0, 255, 0), 2, 8, 0);
					line(view, ppSdR.at(0), ppSdR.at(3), Scalar(0, 0, 255), 2, 8, 0);
					end = clock();
					elapsed_secs5 = double(end - begin) / CLOCKS_PER_SEC;
				}

				//cout << "frame " << frames << " found.  Fps = " << actualFrRate << "   tot : " << elapsed_secs << "   t0 : " << elapsed_secs0 << "   t1 : " << elapsed_secs1 << "   t2 : " << elapsed_secs2 << "   t3 : " << elapsed_secs3 << "   t4 : " << elapsed_secs4 << "   t5 : " << elapsed_secs5 << endl;
				cout << "frame " << frames << endl;
		} else
		{
			//cout << "frame " << frames << " NOT found.   Fps = " << actualFrRate << endl;
			cout << "frame " << frames << " NOT found." << endl;
		}
		

		if (flagPlot) {
			imshow("image", view);
		}
		else
			key = char(0);
	
		key = waitKey(1); // questo sembra responsabile di un rallentamento notevole

	}

#ifdef FROMCAMERA
	error = camera.StopCapture();
	if (error != PGRERROR_OK)
	{
		// This may fail when the camera was removed, so don't show 
		// an error message
	}
	camera.Disconnect();
#endif

	return 0;
}


#endif 


