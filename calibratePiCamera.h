/*  Interactive Whiteboard Project - Wright State University
*   Matt Piekenbrock - Computer Science
*/

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;

double calibratePiCamera(cv::Mat &intrinsic, cv::Mat &distCoeffs, vector <cv::Mat> &rvecs, vector <cv::Mat> &tvecs)
{
	// Declare variables (tweak numBoards to # of images to take)
	int numBoards = 7, numCornersHor = 8, numCornersVer = 6, successes = 0;;
	int nTotalSquares = numCornersHor * numCornersVer;
	cv::Size board_sz = cv::Size(numCornersHor, numCornersVer), size(800, 600);
	cv::Mat image, gray_image;
	vector <vector <cv::Point3f>> object_points, image_points;
	vector <cv::Point3f> corners;

	cout << "got here" << endl;
	cout.flush();

	// Get input frame (image) from video

	cvtColor(image, gray_image, CV_BGR2GRAY);

//	if (!capture.isOpened())  // if not success, exit program
//		cout << "Cannot open video stream" << endl;
//	else if (double fps = capture.get(CV_CAP_PROP_FPS) > 0)
//		cout << "Capturing at " << fps << " fps" << endl;
	cv::VideoCapture capture = cv::VideoCapture(0);
	while (successes < numBoards)
	{
		capture >> image;
		// List of vertices of chessboard corners
		vector <cv::Point3f> obj;
		for (int j = 0; j < nTotalSquares; j++)
			obj.push_back(cv::Point3f(j / numCornersHor, j % numCornersHor, 0.0f));

		bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		// Corners contains pixel coord's of corners that matched the chessboard pattern
		if (found)
		{
//			cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			cvtColor(image, gray_image, CV_BGR2GRAY);
			drawChessboardCorners(gray_image, board_sz, corners, found);
		}

		// Visual feedback
		//cv::resize(image, image, size);
		imshow("Not Gray", image);

		// Space-bar stores results and keeps going for numBoards times
		int key = cv::waitKey(0);
		if (key == ' ' && found != 0)
		{
			image_points.push_back(corners);
			object_points.push_back(obj);
			cout << "calibration image " << successes++ << " stored" << endl;
		} else if (key == 27)
			return 0;
	}
	capture.release();
	cv::calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
	return 0;
}


// Testing
std::vector <cv::Point3f> Create3DChessboardCorners(cv::Size boardSize, float squareSize)
{
	// This function creates the 3D points of your chessboard in its own coordinate system
	std::vector <cv::Point3f> corners;
	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
	return corners;
}
void testingCalibration()
{
	// Specify the number of squares along each dimension of the board.
	// This is actually the number of "inside corners" (where a black square meets a white square).
	// That is, if the board is composed of n x m squares, you would use (n-1, m-1) as the arguments.
	// For example, for a standard checkerboard (8x8 squares), you would use:
	cv::Size boardSize(7, 5);

	float squareSize = 1.f; // This is "1 arbitrary unit"


	cv::Mat image;
	cv::VideoCapture capture = cv::VideoCapture(0);
	capture >> image;
	if (image.empty())
	{
		std::cerr << "Image not read correctly!" << std::endl;
		exit(-1);
	}

	cv::namedWindow("Image View", 1);

	cv::Size imageSize = image.size();

	// Find the chessboard corners
	std::vector <std::vector <cv::Point2f> > imagePoints(1);
	bool found = findChessboardCorners(image, boardSize, imagePoints[0]);
	while (!found)
	{
		std::cerr << "Could not find chess board!" << std::endl;
		cv::imshow("Image View", image);
		found = findChessboardCorners(image, boardSize, imagePoints[0]);
	}

	drawChessboardCorners(image, boardSize, cv::Mat(imagePoints[0]), found);

	std::vector <std::vector <cv::Point3f> > objectPoints(1);
	objectPoints[0] = Create3DChessboardCorners(boardSize, squareSize);

	std::vector <cv::Mat> rotationVectors;
	std::vector <cv::Mat> translationVectors;

	cv::Mat distortionCoefficients = cv::Mat::zeros(8, 1, CV_64F); // There are 8 distortion coefficients
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

	int flags = 0;
	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distortionCoefficients, rotationVectors, translationVectors, flags | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

	std::cout << "RMS: " << rms << std::endl;

	std::cout << "Camera matrix: " << cameraMatrix << std::endl;
	std::cout << "Distortion _coefficients: " << distortionCoefficients << std::endl;

	cv::imshow("Image View", image);
	cv::waitKey(0);
}



//	May not need these
//	intrinsic.ptr <float>(0)[0] = 3.6; // focal length along X for Pi camera
//	intrinsic.ptr <float>(1)[1] = 3.6; // focal length along Y for Pi camera