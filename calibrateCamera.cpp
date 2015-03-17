/*  Interactive Whiteboard Project - Wright State University
*   Matt Piekenbrock - Computer Science
*/

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;

double calibratePiCamera()
{
	// Declare variables (tweak numBoards to # of images to take)
	int numBoards = 7, numCornersHor = 8, numCornersVer = 6, successes = 0;;
	int nTotalSquares = numCornersHor * numCornersVer;
	cv::Size board_sz = cv::Size(numCornersHor, numCornersVer), size(800, 600);
	cv::Mat image, gray_image;
	vector <vector <cv::Point3f>> object_points, image_points;
	vector <cv::Point3f> corners;

	// Get input frame (image) from video
	cv::VideoCapture capture = cv::VideoCapture(0);
	capture >> image;

	// List of vertices
	vector <cv::Point3f> obj;
	for (int j = 0; j < nTotalSquares; j++)
	{obj.push_back(cv::Point3f(j / numCornersHor, j % numCornersHor, 0.0f));}

	while (successes < numBoards)
	{
		cvtColor(image, gray_image, CV_BGR2GRAY);
		bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		// Corners contains pixel coord's of corners that matched the chessboard pattern
		if (found)
		{
			cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray_image, board_sz, corners, found);
		}

		// Visual feedback
		cv::resize(gray_image, gray_image, size);
		imshow("Gray", gray_image);

		// Space-bar stores results and keeps going for numBoards times
		int key = cv::waitKey(1);
		if (key == ' ' && found != 0)
		{
			image_points.push_back(corners);
			object_points.push_back(obj);
			cout << "calibration image " << successes++ << " stored" << endl;
		}
		else if (key == 27)
			return 0;
	}
	// Prepare variables for camera calibration
	cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1), distCoeffs;
	vector <cv::Mat> rvecs, tvecs; // translation and rotation vectors
	intrinsic.ptr <float>(0)[0] = 1; // focal length along X for Pi camera
	intrinsic.ptr <float>(1)[1] = 1; // focal length along Y for Pi camera

	// Release camera image, return lens distortion
	capture.release();
	return cv::calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
}