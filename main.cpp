#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/nonfree/features2d.hpp>

using namespace std;
using namespace cv;

void calibratePiCamera()
{
	int numBoards = 8, numCornersHor = 9, numCornersVer = 6;
	int nTotalSquares = numCornersHor * numCornersVer;
	cv::Size board_sz = cv::Size(numCornersHor, numCornersVer);
	VideoCapture capture = VideoCapture(0);
	vector<vector<Point3f>> object_points; // <-- Should be Chessboard corners
	vector<vector<Point2f>> image_points; // <-- image_points is the location of the corners
	vector<Point2f> corners;
	int successes = 0;
	Mat image, gray_image;
	cv::Size size(800, 600);
	// Input image from video
	capture >> image;

	// List of vertices
	vector<Point3f> obj;
	for (int j = 0; j < nTotalSquares; j++)
	{obj.push_back(Point3f(j / numCornersHor, j % numCornersHor, 0.0f));}

	while (successes < numBoards)
	{
		cvtColor(image, gray_image, CV_BGR2GRAY);
		bool found = findChessboardCorners(image, board_sz, corners,
		                                   CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		// Corners contains pixel coord's of corners that matched the pattern
		// TODO: improve accuracy with cornersSubPix function and tweaking of epsilon
		if (found)
		{
			//cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray_image, board_sz, corners, found);
		}

		cv::resize(image, image, size);
		cv::resize(gray_image, gray_image, size);
		imshow("Original", image);
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
	vector<Mat> rvecs;
	vector<Mat> tvecs;
	intrinsic.ptr<float>(0)[0] = 1; // focal length along X for Pi camera
	intrinsic.ptr<float>(1)[1] = 1; // focal length along X for Pi camera
	calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
	Mat imageUndistorted;
	while (1)
	{
		capture >> image;
		undistort(image, imageUndistorted, intrinsic, distCoeffs);
		cv::resize(image, image, size);
		cv::resize(imageUndistorted, imageUndistorted, size);
		imshow("Supposedly distorted image", image);
		imshow("supposedly undistorted image", imageUndistorted);
		int key = waitKey(1);
		if (key == 27) // Esc key
			break;
	}
	capture.release();
} // void calibrateCamera()

cv::KeyPoint filter_keypoints(vector<cv::KeyPoint> &my_key_list)
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

void detectBlob(cv::Mat &image)
{
	SimpleBlobDetector::Params params;
	params.thresholdStep = 5;
	params.minThreshold = 200;
	params.maxThreshold = 255;
	params.minDistBetweenBlobs = 0;
	params.filterByColor = true;
	params.blobColor = 255;
	params.filterByCircularity = false;
	params.filterByInertia = true;
	params.filterByArea = true;
	params.minArea = 500.0f;
	params.maxArea = 100000.0f;
	params.minInertiaRatio = 0.005; // min/max = circularity, where 0 == line, 1 == circle
	params.maxInertiaRatio = 1;
	params.filterByConvexity = false;
	Ptr<FeatureDetector> blob_detector = new cv::SimpleBlobDetector(params);
	blob_detector->create("SimpleBlob");
	vector<KeyPoint> blob_keypoints, new_keypoints;
	blob_detector->detect(image, blob_keypoints);
	double* blob_area = 0;
	for (auto &elem: blob_keypoints)
	{
		cout << elem.size << endl;
		std::cout.flush();
	}
	//new_keypoints.push_back(filter_keypoints(blob_keypoints));
	drawKeypoints(image, new_keypoints, image, CV_RGB(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	// Debugging
	Size wind_size_DEBUG(800, 600);
	resize(image, image, wind_size_DEBUG);
	namedWindow("Blob detector", CV_GUI_NORMAL);
	imshow("Blob detector", image);
	waitKey(0);
	return;
}

vector<cv::KeyPoint> detectBlobs(std::vector<cv::Mat*> images, int output)
{
	cv::SimpleBlobDetector::Params params;
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

	// Various filter parameters
//	params.blobColor = 240;
//	params.minArea = 4.0f;
//  params.maxArea = 50.0f;
	params.minConvexity = 0.2;
	params.minCircularity = 0.2;
	params.minInertiaRatio = 0.2;

	params.maxConvexity = 1.0;
	params.maxCircularity = 1.0;
	params.maxInertiaRatio = 1.0;

	cv::Ptr<cv::FeatureDetector> blob_detector = new cv::SimpleBlobDetector(params);
	blob_detector->create("SimpleBlob");
	vector<cv::KeyPoint> keypoints_image1, keypoints_image2, keypoints_image3, keypoints_image4, keypoints_r_img5;
	vector<cv::KeyPoint> keypoints_r_img6, keypoints_image7, keypoints_image8, blob_keypoints;
	blob_detector->detect(*images.at(0), keypoints_image1);
	blob_detector->detect(*images.at(1), keypoints_image2);
	blob_detector->detect(*images.at(2), keypoints_image3);
	blob_detector->detect(*images.at(3), keypoints_image4);
	blob_detector->detect(*images.at(4), keypoints_r_img5);
	blob_detector->detect(*images.at(5), keypoints_r_img6);
	blob_detector->detect(*images.at(6), keypoints_image7);
	blob_detector->detect(*images.at(7), keypoints_image8);
	cv::KeyPoint blob_center_image1, blob_center_image2, blob_center_image3, blob_center_image4, blob_center_r_img5;
	cv::KeyPoint blob_center_r_img6, blob_center_image7, blob_center_image8;

//  // Debugging
//  cv::Size wind_size_DEBUG(800, 600);
//  cv::resize(*images.at(5), *images.at(5), wind_size_DEBUG);
//  cv::namedWindow("Blob detector", CV_GUI_NORMAL);
//  cv::imshow("Blob detector", *images.at(5));
//  cv::waitKey(0);

	blob_center_image1 = filter_keypoints(keypoints_image1);
	blob_center_image2 = filter_keypoints(keypoints_image2);
	blob_center_image3 = filter_keypoints(keypoints_image3);
	blob_center_image4 = filter_keypoints(keypoints_image4);
	blob_center_r_img5 = filter_keypoints(keypoints_r_img5);
	blob_center_r_img6 = filter_keypoints(keypoints_r_img6);
	blob_center_image7 = filter_keypoints(keypoints_image7);
	blob_center_image8 = filter_keypoints(keypoints_image8);
	blob_keypoints.push_back(cv::KeyPoint(blob_center_image1.pt.x, blob_center_image1.pt.y, blob_center_image1.size));
	cv::drawKeypoints(*images.at(0), blob_keypoints, *images.at(0), CV_RGB(0, 255, 0),
	                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	blob_keypoints.push_back(cv::KeyPoint(blob_center_image2.pt.x, blob_center_image2.pt.y, blob_center_image2.size));
	cv::drawKeypoints(*images.at(1), blob_keypoints, *images.at(1), CV_RGB(0, 255, 0),
	                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	blob_keypoints.push_back(cv::KeyPoint(blob_center_image3.pt.x, blob_center_image3.pt.y, blob_center_image3.size));
	cv::drawKeypoints(*images.at(2), blob_keypoints, *images.at(2), CV_RGB(0, 255, 0),
	                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	blob_keypoints.push_back(cv::KeyPoint(blob_center_image4.pt.x, blob_center_image4.pt.y, blob_center_image4.size));
	cv::drawKeypoints(*images.at(3), blob_keypoints, *images.at(3), CV_RGB(0, 255, 0),
	                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	blob_keypoints.push_back(cv::KeyPoint(blob_center_r_img5.pt.x, blob_center_r_img5.pt.y, blob_center_r_img5.size));
	cv::drawKeypoints(*images.at(4), blob_keypoints, *images.at(4), CV_RGB(0, 255, 0),
	                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	blob_keypoints.push_back(cv::KeyPoint(blob_center_r_img6.pt.x, blob_center_r_img6.pt.y, blob_center_r_img6.size));
	cv::drawKeypoints(*images.at(5), blob_keypoints, *images.at(5), CV_RGB(0, 255, 0),
	                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	blob_keypoints.push_back(cv::KeyPoint(blob_center_image7.pt.x, blob_center_image7.pt.y, blob_center_image7.size));
	cv::drawKeypoints(*images.at(6), blob_keypoints, *images.at(6), CV_RGB(0, 255, 0),
	                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	blob_keypoints.push_back(cv::KeyPoint(blob_center_image8.pt.x, blob_center_image8.pt.y, blob_center_image8.size));
	cv::drawKeypoints(*images.at(7), blob_keypoints, *images.at(7), CV_RGB(0, 255, 0),
	                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	cout << "Keypoints " << blob_keypoints.size() << endl;
	if (output)
	{
		cv::Size wind_size(800, 600);
		cv::resize(*images.at(0), *images.at(0), wind_size);
		cv::resize(*images.at(1), *images.at(1), wind_size);
		cv::resize(*images.at(2), *images.at(2), wind_size);
		cv::resize(*images.at(3), *images.at(3), wind_size);
		cv::resize(*images.at(4), *images.at(4), wind_size);
		cv::resize(*images.at(5), *images.at(5), wind_size);
		cv::resize(*images.at(6), *images.at(6), wind_size);
		cv::resize(*images.at(7), *images.at(7), wind_size);
		cv::namedWindow("Blob detector", CV_GUI_NORMAL);
		cv::imshow("Blob detector", *images.at(0));
		cv::waitKey(0);
		cv::imshow("Blob detector", *images.at(1));
		cv::waitKey(0);
		cv::imshow("Blob detector", *images.at(2));
		cv::waitKey(0);
		cv::imshow("Blob detector", *images.at(3));
		cv::waitKey(0);
		cv::imshow("Blob detector", *images.at(4));
		cv::waitKey(0);
		cv::imshow("Blob detector", *images.at(5));
		cv::waitKey(0);
		cv::imshow("Blob detector", *images.at(6));
		cv::waitKey(0);
		cv::imshow("Blob detector", *images.at(7));
		cv::waitKey(0);
	}
	return blob_keypoints;
}

std::vector<cv::Point2f> scanThresholdedImage(cv::Mat &image)
{
	std::vector<cv::Point2f> detected_object;
	int count = 0;
	for (int row = 0; row < image.rows; ++row)
	{
		uchar* data = image.ptr(row);
		for (int column = 0; column < image.cols; column++)
		{
			if (data[column] > 242 && count < 8)
			{
				detected_object.push_back(cv::Point2f((float) row, (float) column));
				++count;
			}
			//else
			//{std::cout << "R: " << row << " C: " << column << " Value: " << (int) data[column] << std::endl;}
		}
	}
	return detected_object;
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool my_intersection(cv::Point2d o1, cv::Point2d p1, cv::Point2d o2, cv::Point2d p2, cv::Point2d &r)
{
	cv::Point2d x = o2 - o1;
	cv::Point2d d1 = p1 - o1;
	cv::Point2d d2 = p2 - o2;
	double cross = d1.x * d2.y - d1.y * d2.x;
	if (abs(cross) < /*EPS*/1e-5)
		return false;
	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
}

Mat rotateImage(const Mat &source, double angle)
{
	Point2f src_center(source.cols / 2.0F, source.rows / 2.0F);
	Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
	Mat dst;
	warpAffine(source, dst, rot_mat, source.size());
	return dst;
}

/* Left and Right determined from a top-down view */
int main()
{

	cv::Size size(800, 600);
	//calibratePiCamera();
	std::vector<cv::Mat*> camera_vector_angle_R, camera_vector_angle_L;
	cv::Mat r_imgs[9], l_imgs[9];
	cv::Mat l_img1, l_img2, l_img3, l_img4, l_img5, l_img6, l_img7, l_img8, imageL_output;

	std::string img_dir ="";
	std::string folder_base = "images1";
	cv::Mat img;
	// Right
	for (int i = 1; i <= 8; ++i)
	{
		img_dir = folder_base+"/r_img"+std::to_string(i)+".jpg";
		r_imgs[i]  = cv::imread(img_dir, 1);
		cv::threshold(r_imgs[i] , r_imgs[i] , 60, 255, 3);
		cv::cvtColor(r_imgs[i], r_imgs[i], CV_RGB2GRAY);
		camera_vector_angle_R.push_back(&r_imgs[i]);
	}
	// Left
	for (int i = 1; i <= 8; ++i)
	{
		img_dir = folder_base+"/l_img"+std::to_string(i)+".jpg";
		l_imgs[i] = cv::imread(img_dir, 1);
		cv::threshold(l_imgs[i], l_imgs[i], 60, 255, 3);
		cv::cvtColor(l_imgs[i], l_imgs[i], CV_RGB2GRAY);
		camera_vector_angle_L.push_back(&l_imgs[i]);
	}

	//std::vector<cv::Point2f> object_points_image1, object_points_image2;
	std::vector<cv::KeyPoint> detected_objs_R, detected_obj_L;
	std::cout << "got here" << std::endl;
	std::cout.flush();

	detected_objs_R = detectBlobs(camera_vector_angle_R, 0);
	detected_obj_L = detectBlobs(camera_vector_angle_L, 0);
	cout << "\n=================== Blob Results (R) ===================" << endl;
	for (auto &blob_keypoint: detected_objs_R)
	{cout << blob_keypoint.pt;}
	cout << "\n=================== Blob Results (L) ===================" << endl;
	for (auto &blob_keypoint: detected_obj_L)
	{cout << blob_keypoint.pt;}
//
	// Convert KeyPoints to points vector
	std::vector<cv::Point2f> pointsR, pointsL;
	cv::KeyPoint::convert(detected_objs_R, pointsR);
	cv::KeyPoint::convert(detected_obj_L, pointsL);

	// left points == 'First' image, right points == 'Second' image
	cv::Mat fundamental_matrix = cv::findFundamentalMat(pointsL, pointsR, CV_FM_8POINT);

	cv::Mat out_imgL, out_imgR, out_imgLR;
	cout << "fundamental matrix: " << endl;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			cout << fundamental_matrix.at<float>(i, j) << endl;
		}
	}

	cout << "\nPrincipal Point: (" << fundamental_matrix.at<float>(1, 3) << "," << fundamental_matrix.at<float>(2,
	                                                                                                            3) << ")\n";
	//outputimage3 = cv::imread("objective_plane.jpg", 1);
	//transpose(outputimage3, outputimage3);
	//out_img = rotateImage(out_img, 90);


//	cv::computeCorrespondEpilines(cv::Mat(pointsL), 1, fundamental_matrix, epilines_as_viewed_from_right_cam);
//	cout << "====================== Epilines (ax + by + c = 0) [R]: ======================" << endl;
//	for (auto &elem: epilines_as_viewed_from_right_cam)
//	{cout << elem;}
//	cout.flush(); cout << endl;
//
//	// draw the left points corresponding epipolar lines in left image
//	cv::computeCorrespondEpilines(cv::Mat(pointsR), 2, fundamental_matrix, epilines_as_viewed_from_left_cam);
//	cout << "====================== Epilines (ax + by + c = 0) [L]: ======================" << endl;
//	for (auto &elem: epilines_as_viewed_from_left_cam)
//	{cout << elem;}

	cout.flush();
	cout << endl;


	/* -c = ax + by */
	/* x = (-by - c)/a */
	/* y =( -c - ax)/b */



	/* line = ax + by + c = 0 */
//	double x_interceptR = (-(*abcR)[2])/(*abcR)[0];
//	double y_interceptR = (-(*abcR)[2])/(*abcR)[1];

	std::vector<cv::Vec3f> epilines_as_viewed_from_right_cam, epilines_as_viewed_from_left_cam;

	cv::computeCorrespondEpilines(cv::Mat(pointsL), 1, fundamental_matrix, epilines_as_viewed_from_right_cam);
	cv::computeCorrespondEpilines(cv::Mat(pointsR), 2, fundamental_matrix, epilines_as_viewed_from_left_cam);

	std::vector<cv::Vec3f>::const_iterator abcR = epilines_as_viewed_from_right_cam.begin();
	std::vector<cv::Vec3f>::const_iterator abcL = epilines_as_viewed_from_left_cam.begin();

	double x_interceptR = -(*abcR)[2] / (*abcR)[1];
	double y_interceptR = -(*abcR)[2] / (*abcR)[1];
//
	double x_interceptL = (-(*abcL)[2]) / (*abcL)[0];
	double y_interceptL = (-(*abcL)[2]) / (*abcL)[1];

	// draw the epipolar li
	// ne between first and last column
	int i = 0;
	std::vector<cv::Point2d> left_epilines;
	std::vector<cv::Point2d> right_epilines;

	while (abcL != epilines_as_viewed_from_left_cam.end())
	{
		x_interceptL = (-(*abcL)[2]) / (*abcL)[0];
		y_interceptL = (-(*abcL)[2]) / (*abcL)[1];
		out_imgL = *camera_vector_angle_L.at(i);
		left_epilines.push_back(cv::Point2d(0, -(*abcL)[2] / (*abcL)[1]));
		left_epilines.push_back(cv::Point2d(out_imgR.cols, -((*abcL)[2] + (*abcL)[0] * out_imgR.cols) / (*abcL)[1]));
		/* pt1 = (0, y_intercept)
		*   pt2 = (MAX, (-(c + [a*max_width])/b)*/

		/* -c = ax + by */
		/* x = (-by - c)/a */
		/* y = -(c + ax)/b */
		cv::line(out_imgL, cv::Point2d(0, -(*abcL)[2] / (*abcL)[1]),
		         cv::Point2d(out_imgL.cols, -((*abcL)[2] + (*abcL)[0] * out_imgL.cols) / (*abcL)[1]), CV_RGB(0, 255, 0), 2,
		         CV_AA, 0);
		//cv::line(out_imgL, cv::Point2d(0, y_interceptL), cv::Point2d(x_interceptL, 0), CV_RGB(0, 255, 0), 2, CV_AA, 0);
		abcL++;
		i++;
		cv::namedWindow("Epiline L", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
		cv::resize(out_imgL, out_imgL, size);
		cv::imshow("Epiline L", out_imgL);
		cv::waitKey(0);
	}
	i = 0;
	while (abcR != epilines_as_viewed_from_right_cam.end())
	{
		out_imgR = *camera_vector_angle_R.at(i);
		right_epilines.push_back(cv::Point2d(0, -(*abcR)[2] / (*abcR)[1]));
		right_epilines.push_back(cv::Point2d(out_imgR.cols, -((*abcR)[2] + (*abcR)[0] * out_imgR.cols) / (*abcR)[1]));
		cv::line(out_imgR, cv::Point2d(0, -(*abcR)[2] / (*abcR)[1]),
		         cv::Point2d(out_imgR.cols, -((*abcR)[2] + (*abcR)[0] * out_imgR.cols) / (*abcR)[1]), CV_RGB(0, 255, 0), 2, CV_AA,
		         0);
//		x_interceptR = -(*abcR)[2] / (*abcR)[1];
//		y_interceptR = -(*abcR)[2] / (*abcR)[1];
//		cv::line(out_imgR, cv::Point2d(0, y_interceptR), cv::Point2d(x_interceptR, 0), CV_RGB(0, 255, 0), 2, CV_AA, 0);
		abcR++;
		i++;
		cv::namedWindow("Epiline R", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
		cv::resize(out_imgR, out_imgR, size);
		cv::imshow("Epiline R", out_imgR);
		cv::waitKey(0);
	}

//	cv::line(out_imgR, cv::Point2d(0,y_interceptR), cv::Point2d(x_intercept, 0), CV_RGB(0, 255, 0), 2,
//	         CV_AA, 0);
//	cv::namedWindow("Epiline R", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//	cv::namedWindow("Epiline L", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//	cv::resize(l_img1, l_img1, size);
//	cv::resize(out_imgL, out_imgL, size);
//	cv::imshow("Epiline R", l_img2);
//	cv::imshow("Epiline L", out_imgL);
//	cv::waitKey(0);


//	cv::Point2d intersection(0.0, 0.0);
//	my_intersection(left_epilines.at(0), left_epilines.at(1), right_epilines.at(0), left_epilines.at(1), intersection);
//	cv::circle(out_imgR, intersection, 5, CV_RGB(0, 255, 0), 3);
//	cv::namedWindow("Inter", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//	cv::resize(out_imgR, out_imgR, size);
//	cv::imshow("Inter", out_imgR);
//	cv::waitKey(0);
//	for (; abcR != epilines_as_viewed_from_right_cam.end(); ++abcR, ++abcL)
//	{
//		double right_image_x = -(*abcR)[2] / (*abcR)[1]; //
//		double right_image_y = -((*abcR)[2] + (*abcR)[0] * out_imgLR.cols) / (*abcL)[1];
//		double left_image_x = -(*abcL)[2] / (*abcL)[1];
//		double left_image_y = -((*abcL)[2] + (*abcL)[0] * out_imgLR.cols) / (*abcL)[1];
//		cv::Point2d pt1(left_image_x, 0.0);
//		cv::Point2d pt2(0.0, left_image_y);
//		cv::Point2d pt4(right_image_x, 0.0);
//		cv::Point2d pt3(0.0, right_image_y);
//
//
//		cv::namedWindow("Draw Epiline", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//		cv::resize(out_imgLR, out_imgLR, size);
//		cv::imshow("Epilines", out_imgLR);
//		cv::waitKey(50000);
//
//		//my_intersection(pt1, pt2, pt3, pt4, intersection);
//		//cout << intersection << endl;
//	}

//	Size image_size(r_img1.rows, r_img2.cols);
	//stereoRectifyUncalibrated(pointsR, pointsL,  fundamental_matrix,  image_size,  H1,  H2,  5);


//	cv::Mat rectified1(l_img1.size(), l_img1.type());
//	cv::warpPerspective(l_img1, rectified1, H1, l_img1.size());
//	cv::imwrite("rectified1.png", rectified1);
//
//	cv::Mat rectified2(r_img1.size(), r_img1.type());
//	cv::warpPerspective(r_img1, rectified2, H2, r_img1.size());
//	cv::imwrite("rectified2.png", rectified2);
//
//	out_imgLR = l_img1;
//	cv::namedWindow("Epilines", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//	cv::resize(out_imgLR, out_imgLR, size);
//	cv::imshow("Epilines", out_imgLR);
//	cv::waitKey(50000);
//	std::cout << "finished";
//
//  //cv::threshold( image_output, image_output, 242, 255,0); // 0 = Binary threshold

	return 0;
}

/* Unused code */
// Compute Homography
//  std::vector<unsigned char> inliers(points1.size(), 0);
//  cv::Mat homography = cv::findHomography(cv::Mat(points1), cv::Mat(points2), inliers);
//  std::vector<cv::Point2f>::const_iterator itPts = points1.begin();
//  std::vector<unsigned char>::const_iterator o_inliers = inliers.begin();
//  while (itPts != points1.end())
//  {
//    if (*o_inliers)
//    {
//      cv::circle(outputimage3, *itPts, 3, CV_RGB(0, 255, 0), 2);
//    }
//    ++itPts;
//    ++o_inliers;
//  }

// Display the images with points and epipolar lines

//
//  cv::namedWindow("Homography test", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//  cv::resize(outputimage3, outputimage3, size);
//  cv::imshow("Homography test", outputimage3);