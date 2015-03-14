#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>

using namespace std;
using namespace cv;
void calibratePiCamera()
{
	int numBoards = 7, numCornersHor = 8, numCornersVer = 6;
	int nTotalSquares = numCornersHor * numCornersVer;
	cv::Size board_sz = cv::Size(numCornersHor, numCornersVer);
	cv::FileStorage distCoeffs_file("distCoeffs.txt", cv::FileStorage::WRITE);
	VideoCapture capture = VideoCapture(0);
	vector <vector <Point3f> > object_points; // Should be Chessboard corners
	vector <vector <Point2f> > image_points; 
	vector <Point2f> corners;
	int successes = 0;
	Mat image, gray_image;
	cv::Size size(800, 600);
	// Input image from video
	capture >> image;

	// /usr/local/bin/clion-140.2310.6/bin:
	// List of vertices
	vector <Point3f> obj;
	for (int j = 0; j < nTotalSquares; j++)
	{obj.push_back(Point3f(j / numCornersHor, j % numCornersHor, 0.0f));}

	while (successes < numBoards)
	{
		cvtColor(image, gray_image, CV_BGR2GRAY);
		bool found = findChessboardCorners(image, board_sz, corners,
		                                   CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		// Corners contains pixel coord's of corners that matched the pattern
		if (found)
		{
			cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray_image, board_sz, corners, found);
		}

		cv::resize(image, image, size);
		cv::resize(gray_image, gray_image, size);
		//imshow("Original", image);
		imshow("Gray", gray_image);
		capture >> image;
		int key = waitKey(1);
		if (key == 27) // Esc key
			return;
		if (key == ' ' && found != 0) // Space-bar stores results and keeps going for pre-set numBoards
		{
			image_points.push_back(corners);
			object_points.push_back(obj);
			printf("Snap stored!");
			++successes;
			if (successes >= numBoards)
				break;
		}
	}
	Mat intrinsic = Mat(3, 3, CV_32FC1); // intrinsic parameters of camera!
	Mat distCoeffs;
	vector <Mat> rvecs;
	vector <Mat> tvecs;
	intrinsic.ptr <float>(0)[0] = 1; // focal length along X for Pi camera
	intrinsic.ptr <float>(1)[1] = 1; // focal length along X for Pi camera
	calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
	file << distCoeffs_file;  


	// Mat imageUndistorted;
	// while (1)
	// {
	// 	capture >> image;
	// 	undistort(image, imageUndistorted, intrinsic, distCoeffs);
	// 	cv::resize(image, image, size);
	// 	cv::resize(imageUndistorted, imageUndistorted, size);
	// 	imshow("Supposedly distorted image", image);
	// 	imshow("supposedly undistorted image", imageUndistorted);
	// 	int key = waitKey(1);
	// 	if (key == 27) // Esc key
	// 		break;
	// }
	capture.release();
} // void calibrateCamera()

int main()
{
	calibratePiCamera(); 
	return 0; 
}
