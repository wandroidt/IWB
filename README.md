# IWB
Interactive Whiteboard Project


Important Notes:

Fundamental Matrix estimation is sensitive to quality of matches, outliers etc. It becomes worse when all selected matches lie on the same plane.

void projectPoints(InputArray objectPoints, InputArray rvec, InputArray tvec, InputArray cameraMatrix, InputArray distCoeffs, OutputArray imagePoints)
==> "The function computes projections of 3D points to the image plane given intrinsic and extrinsic camera parameters."
