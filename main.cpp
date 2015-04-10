#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "calibratePiCamera.h"

using namespace std;
using namespace cv;

typedef unsigned int uint;

KeyPoint filter_keypoints(vector <KeyPoint> &my_key_list)
{
	double min_size = 0;
	int index = 0;
	if (my_key_list.size() == 0)
	{cout << "No Blobs detected!! " << endl;} else
	{cout << "Keypoints found..." << endl;}
	for (int i = 0; i < my_key_list.size(); ++i)
	{
		if (my_key_list[i].size > min_size)
		{
			min_size = my_key_list[i].size;
			index = i;
		}
	}
	return my_key_list[index];
}

Ptr <FeatureDetector> getBlobDetector()
{
	SimpleBlobDetector::Params params;
	// Thresholding and blob merging params
	params.thresholdStep = 1;
	params.minThreshold = 120;
	params.maxThreshold = 255;
	params.minDistBetweenBlobs = 100;

	// Which filters to apply
	params.filterByColor = false;
	params.filterByInertia = true;
	params.filterByArea = false;
	params.filterByCircularity = true;
	params.filterByConvexity = true;

	// Other filter parameters
	params.minConvexity = 0.2;
	params.maxConvexity = 1.0;
	params.minCircularity = 0.2;
	params.maxCircularity = 1.0;
	params.minInertiaRatio = 0.2;
	params.maxInertiaRatio = 1.0;

	Ptr <FeatureDetector> blob_detector = new SimpleBlobDetector(params);
	blob_detector->create("SimpleBlob");
	return blob_detector;
}

bool detectBlob(Mat &image, KeyPoint& keypoint,Ptr <FeatureDetector> blob_detector)
{
	vector <KeyPoint> blob_keypoints = vector <KeyPoint>();
	blob_detector->detect(image, blob_keypoints);
	drawKeypoints(image, blob_keypoints, image, CV_RGB(0, 255, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	if (blob_keypoints.empty())
		return false;
	else
		keypoint = filter_keypoints(blob_keypoints);
	return true;
}

vector <KeyPoint>* detectBlobs(Mat* images, uint nImages, int output)
{
	// Initialize variables
	Ptr <FeatureDetector> blob_detector = getBlobDetector();
	vector <KeyPoint>* blob_keypoints = new vector <KeyPoint>();
	KeyPoint blob_keypoint = KeyPoint();

	// Detect all blobs that meet blob detector parameter criteria
	for (uint i = 0; i < nImages; ++i)
		if (detectBlob(images[i], blob_keypoint, blob_detector))
			blob_keypoints->push_back(blob_keypoint);

	// If output flag is specified, display detected blobs for each image
	if (output)
	{
		Size wind_size(800, 600);
		namedWindow("Blob detector", CV_GUI_NORMAL);
		for (uint i = 0; i < blob_keypoints->size(); ++i)
		{
			vector <KeyPoint> blob = vector <KeyPoint>(1);
			blob.push_back(blob_keypoints->at(i));
			drawKeypoints(images[i], blob, images[i], CV_RGB(0, 255, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			resize(images[i], images[i], wind_size);
			imshow("Blob detector", images[i]);
			waitKey(0);
		}
	}
	return blob_keypoints;
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool my_intersection(Point2d o1, Point2d p1, Point2d o2, Point2d p2, Point2d &r)
{
	Point2d x = o2 - o1;
	Point2d d1 = p1 - o1;
	Point2d d2 = p2 - o2;
	double cross = d1.x * d2.y - d1.y * d2.x;
	if (abs(cross) < /*EPS*/1e-5)
		return false;
	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
}

vector <cv::Point2f> getBoardDimensions()
{
	vector <cv::Point2f> corners;
	cv::Size size(800, 600);
	cv::Mat image, gray_image;
	cv::KeyPoint keypoint = KeyPoint();
	bool found = 0;
	Ptr<FeatureDetector> blob_detector = getBlobDetector();

	// Stream input frames (image) from video
	cv::VideoCapture capture = cv::VideoCapture(0);
	capture >> image;

	// Visual feedback
	cvtColor(image, gray_image, CV_BGR2GRAY);
	cv::resize(gray_image, gray_image, size);
	imshow("Gray", gray_image);

	uint i = 0;
	while (i < 4)
	{
		int key = cv::waitKey(1);
		// Space-bar stores corner results
		if (detectBlob(gray_image, keypoint, blob_detector) && key == ' ')
		{
			cout << "corner " << i++ << " stored" << endl;
			corners.push_back(keypoint.pt);
		}
		capture.release();
	}
	return corners;
}


Mat rotateImage(const Mat &source, double angle)
{
	Point2f src_center(source.cols / 2.0F, source.rows / 2.0F);
	Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
	Mat dst;
	warpAffine(source, dst, rot_mat, source.size());
	return dst;
}

void getImages(char* src_folder, char* image_pattern, Mat* images, uint nImages, bool grayscale)
{
	char img_path[1024];
	for (uint i = 0; i < nImages; ++i)
	{
		sprintf(img_path, "%s/%s%u.jpg", src_folder, image_pattern, i);
		cout << img_path << endl;
		images[i] = imread(img_path, 1);
		threshold(images[i], images[i], 60, 255, 3);
		if (grayscale)
			cvtColor(images[i], images[i], CV_RGB2GRAY);
	}
}
/* Left and Right determined from a top-down view */
int main()
{
	Size size(800, 600);
	const uint nImages = 8;
	cv::Mat r_imgs[nImages], l_imgs[nImages], nr_imgs[nImages], nl_imgs[nImages];

	// Load input images
	getImages((char*) "images", (char*) "r_img", r_imgs, 8, false);
	getImages((char*) "images", (char*) "l_img", l_imgs, 8, false);
	getImages((char*) "images", (char*) "r_img", nr_imgs, 8, true);
	getImages((char*) "images", (char*) "l_img", nl_imgs, 8, true);

	cout << "Got here " << endl;

	// BEGIN Camera calibration **EXPERIMENTAL**
//	cv::Mat& intrinsic = *new cv::Mat(3, 3, CV_32FC1), &distCoeffs =  *new cv::Mat(5, 1, CV_32FC1);
//	vector <cv::Mat>& rvecs = *new vector <cv::Mat>(), &tvecs = *new vector <cv::Mat>();
//	calibratePiCamera(intrinsic, distCoeffs, rvecs, tvecs);
//	testingCalibration();
//	for (uint i =0; i < nImages; ++i)
//	{
//		undistort(r_imgs[i], nr_imgs[i], intrinsic, distCoeffs);
//		undistort(l_imgs[i], nl_imgs[i], intrinsic, distCoeffs);
//	}
	// END Camera calibration **EXPERIMENTAL**

	// Detect corresponding IR pen blobs ( use 0 for no output, 1 for output)
	vector <KeyPoint>* detected_objs_L = detectBlobs(nl_imgs, nImages, 0);
	vector <KeyPoint>* detected_objs_R = detectBlobs(nr_imgs, nImages, 0);

	cout << "Got here 3" << endl;

	if (detected_objs_L->size() != detected_objs_R->size())
		exit(1);

//	for (uint i = 0; i < detected_objs_L->size(); ++i)
//		undistortPoints(detected_objs_L, detected_objs_L, intrinsic, distCoeffs, rvecs, tvecs);

	cout << "\n=================== Normalized Blob Results (L) ===================" << endl;
	for (auto &blob_keypoint: *detected_objs_L)
	{cout << blob_keypoint.pt;}
	cout << "\n=================== Normalized Blob Results (R) ===================" << endl;
	for (auto &blob_keypoint: *detected_objs_R)
	{cout << blob_keypoint.pt;}

	// Convert KeyPoints to points vector
	vector <Point2f> pointsL, pointsR;
	KeyPoint::convert(*detected_objs_L, pointsL);
	KeyPoint::convert(*detected_objs_R, pointsR);

	// left points == 'First' image, right points == 'Second' image
	Mat fundamental_matrix = findFundamentalMat(pointsL, pointsR, CV_FM_8POINT);
	Mat out_imgL, out_imgR;
	vector <Vec3f> epilines_as_viewed_from_right_cam, epilines_as_viewed_from_left_cam;

	computeCorrespondEpilines(Mat(pointsL), 1, fundamental_matrix, epilines_as_viewed_from_right_cam);
	computeCorrespondEpilines(Mat(pointsR), 2, fundamental_matrix, epilines_as_viewed_from_left_cam);

	vector <Vec3f>::const_iterator abcR = epilines_as_viewed_from_right_cam.begin();
	vector <Vec3f>::const_iterator abcL = epilines_as_viewed_from_left_cam.begin();

	uint i = 0;
	float y_intercept_L = 0, y_intercept_R = 0;
	vector <Point2d> left_epilines, right_epilines;
	out_imgL = l_imgs[0], out_imgR = r_imgs[0];
	while (abcL != epilines_as_viewed_from_left_cam.end())
	{
		y_intercept_L = (-(*abcL)[2]) / (*abcL)[1];
		y_intercept_R = -((*abcL)[2] + (*abcL)[0] * r_imgs[i].cols) / (*abcL)[1];
		out_imgL = l_imgs[i++];
		left_epilines.push_back(Point2d(0, y_intercept_L));
		left_epilines.push_back(Point2d(out_imgL.cols, y_intercept_R));
		line(out_imgL, Point2d(0, y_intercept_L), Point2d(out_imgL.cols, y_intercept_R), CV_RGB(0, 255, 0), 2, CV_AA, 0);
		namedWindow("Epiline L", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
		resize(out_imgL, out_imgL, size);
		imshow("Epiline L", out_imgL);
		waitKey(0);
		abcL++;
	}
	i = 0;
	while (abcR != epilines_as_viewed_from_right_cam.end())
	{
		out_imgR = r_imgs[i++];
		y_intercept_L = (-(*abcR)[2]) / (*abcR)[1];
		y_intercept_R = -((*abcR)[2] + (*abcR)[0] * l_imgs[i].cols) / (*abcR)[1];
		right_epilines.push_back(Point2d(0, y_intercept_L));
		right_epilines.push_back(Point2d(out_imgR.cols, y_intercept_R));
		line(out_imgR, Point2d(0, y_intercept_L), Point2d(out_imgR.cols, y_intercept_R), CV_RGB(0, 255, 0), 2, CV_AA, 0);
		namedWindow("Epiline R", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
		resize(out_imgR, out_imgR, size);
		imshow("Epiline R", out_imgR);
		waitKey(0);
		abcR++;
	}

	Point2d intersection(0.0, 0.0);
	my_intersection(left_epilines.at(0), left_epilines.at(1), right_epilines.at(0), left_epilines.at(1), intersection);

	return 0;
}


//	line(out_imgR, Point2d(0,y_interceptR), Point2d(x_intercept, 0), CV_RGB(0, 255, 0), 2,
//	         CV_AA, 0);
//	namedWindow("Epiline R", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//	namedWindow("Epiline L", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//	resize(l_img1, l_img1, size);
//	resize(out_imgL, out_imgL, size);
//	imshow("Epiline R", l_img2);
//	imshow("Epiline L", out_imgL);
//	waitKey(0);

//	circle(out_imgR, intersection, 5, CV_RGB(0, 255, 0), 3);
//	namedWindow("Inter", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//	resize(out_imgR, out_imgR, size);
//	imshow("Inter", out_imgR);
//	waitKey(0);
//	for (; abcR != epilines_as_viewed_from_right_cam.end(); ++abcR, ++abcL)
//	{
//		double right_image_x = -(*abcR)[2] / (*abcR)[1]; //
//		double right_image_y = -((*abcR)[2] + (*abcR)[0] * out_imgLR.cols) / (*abcL)[1];
//		double left_image_x = -(*abcL)[2] / (*abcL)[1];
//		double left_image_y = -((*abcL)[2] + (*abcL)[0] * out_imgLR.cols) / (*abcL)[1];
//		Point2d pt1(left_image_x, 0.0);
//		Point2d pt2(0.0, left_image_y);
//		Point2d pt4(right_image_x, 0.0);
//		Point2d pt3(0.0, right_image_y);
//
//
//		namedWindow("Draw Epiline", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//		resize(out_imgLR, out_imgLR, size);
//		imshow("Epilines", out_imgLR);
//		waitKey(50000);
//

//	}

//	Size image_size(r_img1.rows, r_img2.cols);
	//stereoRectifyUncalibrated(pointsR, pointsL,  fundamental_matrix,  image_size,  H1,  H2,  5);



//	out_imgLR = l_img1;
//	namedWindow("Epilines", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//	resize(out_imgLR, out_imgLR, size);
//	imshow("Epilines", out_imgLR);
//	waitKey(50000);
//	cout << "finished";
//
//  //threshold( image_output, image_output, 242, 255,0); // 0 = Binary threshold



/* Unused code */
// Compute Homography
//  vector<unsigned char> inliers(points1.size(), 0);
//  Mat homography = findHomography(Mat(points1), Mat(points2), inliers);
//  vector<Point2f>::const_iterator itPts = points1.begin();
//  vector<unsigned char>::const_iterator o_inliers = inliers.begin();
//  while (itPts != points1.end())
//  {
//    if (*o_inliers)
//    {
//      circle(outputimage3, *itPts, 3, CV_RGB(0, 255, 0), 2);
//    }
//    ++itPts;
//    ++o_inliers;
//  }

// Display the images with points and epipolar lines

//
//  namedWindow("Homography test", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//  resize(outputimage3, outputimage3, size);
//  imshow("Homography test", outputimage3);
//	Mat rectified1(l_img1.size(), l_img1.type());
//	warpPerspective(l_img1, rectified1, H1, l_img1.size());
//	imwrite("rectified1.png", rectified1);
//
//	Mat rectified2(r_img1.size(), r_img1.type());
//	warpPerspective(r_img1, rectified2, H2, r_img1.size());
//	imwrite("rectified2.png", rectified2);
//
//
//computeCorrespondEpilines(Mat(pointsL), 1, fundamental_matrix, epilines_as_viewed_from_right_cam);
//cout << "====================== Epilines (ax + by + c = 0) [R]: ======================" << endl;
//for (auto &elem: epilines_as_viewed_from_right_cam)
//{cout << elem;}
//cout.flush(); cout << endl;
//
//// draw the left points corresponding epipolar lines in left image
//computeCorrespondEpilines(Mat(pointsR), 2, fundamental_matrix, epilines_as_viewed_from_left_cam);
//cout << "====================== Epilines (ax + by + c = 0) [L]: ======================" << endl;
//for (auto &elem: epilines_as_viewed_from_left_cam)
//{cout << elem;}
//
//cout.flush();
//cout << endl;

/* -c = ax + by */
/* x = (-by - c)/a */
/* y =( -c - ax)/b */

/* line = ax + by + c = 0 */
/* pt1 = (0, y_intercept)
*   pt2 = (MAX, (-(c + [a*max_width])/b)*/

/* -c = ax + by */
/* x = (-by - c)/a */
/* y = -(c + ax)/b */
/* -((*abcL)[2] + (*abcL)[0] * out_imgL.cols) / (*abcL)[1])*/
/* -(c+ax/b)*/
//	double x_interceptR = (-(*abcR)[2])/(*abcR)[0];
//	double y_interceptR = (-(*abcR)[2])/(*abcR)[1];
//vector <Point2f> scanThresholdedImage(Mat &image)
//{
//	vector <Point2f> detected_object;
//	int count = 0;
//	for (int row = 0; row < image.rows; ++row)
//	{
//		uchar* data = image.ptr(row);
//		for (int column = 0; column < image.cols; column++)
//		{
//			if (data[column] > 242 && count < 8)
//			{
//				detected_object.push_back(Point2f((float) row, (float) column));
//				++count;
//			}
//			//else
//			//{cout << "R: " << row << " C: " << column << " Value: " << (int) data[column] << endl;}
//		}
//	}
//	return detected_object;
//}