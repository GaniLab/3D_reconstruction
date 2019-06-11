/* This program is intended to reconstruct 3D object points based on perspective projection concept
 * reconstruction is based on available projection matrix from control points
 * additional object points (movable) can be added as much as we want and 3D reconstructed  based on projection matrix from control points
 * numbers of images and its projection matrices needed for finding conrol points location are 2 and 2 respectively
 * All input data is in .txt file format
*/
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

//3D reconstruction using stereo camera concept
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

// Function prototyping

// 2D image control points extractor from text file data.
vector<Point2d> image_points(ifstream& csvImage);

// printing triangulated 3D points into .txt file format
void print_points3D(ofstream& csvTriangulation, Mat& points4DTransposed, Mat& points3D);

// printing rotation vectors into text file
void printRvecs(ofstream& csvRvecs, Mat& rvecs);

// printing translation vectors into text file
void printTvecs(ofstream& csvTvecs, Mat& tvecs);

// printing 3x3 rotation matrix in rodrigues form into text file
void printRodriguesRotation(ofstream& csvR, Mat& rvec, Mat& R);

// create mat object for extrinsic matrix
Mat extrinsic_matrix(Mat& R, Mat& tvec);

// printing extrinsic matrix into text file
void print_extrinsic(ofstream& csvExtrinsicMatrix, Mat& E);

// create mat object for projection matrix
Mat projection_matrix(Mat& camera_matrix, Mat& extrinstic_matrix);

// printing projection matrix into text file
void print_projection_matrix(ofstream& csvProj, Mat& P);

int main(int argc, char **argv)
{
	// 2d image 1 with control points
	// taken at distance 100 mm to the right (positive) of camera coordinate in X direction
	ifstream im1;

	im1.open("single_X100.txt");

	vector<Point2d> image1 = image_points(im1);

	im1.close();

	cout << " image points 1 : " << endl << image1 << endl;

	cout << endl;


	// 2d image 2 with control points
	// taken at distance 100 mm to the left (negative) of camera coordinate in X direction
	ifstream im2;

	im2.open("single_X-100.txt");

	vector<Point2d> image2 = image_points(im2);

	im2.close();

	cout << " image points 2 : " << endl << image2 << endl;

	cout << endl;


	// projection matrix 1
	// taken from control points at distance 100 mm to the right (positive) of camera coordinate in X direction
	ifstream csvProj1("Projection_Matrix_X100.txt");

	double arrayProj1[3][4];

	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 4; col++)
		{
			csvProj1 >> arrayProj1[row][col];

		}

	}

	Mat proj1 = Mat(3, 4, CV_64F, arrayProj1);

	cout << "Projection matrix 1 = " << endl << proj1 << endl;

	csvProj1.close();

	cout << endl;


	// projection matrix 2
	// taken from control points at distance 100 mm to the left (negative) of camera coordinate in X direction
	ifstream csvProj2("Projection_Matrix_X-100.txt");

	double arrayProj2[3][4];

	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 4; col++)
		{
			csvProj2 >> arrayProj2[row][col];

		}

	}

	Mat proj2 = Mat(3, 4, CV_64F, arrayProj2);

	cout << "Projection matrix 1 = " << endl << proj2 << endl;

	csvProj2.close();

	cout << endl;


	// solve stereo triangulation
	Mat points4D;

	Mat points4DTransposed;

	triangulatePoints(proj1, proj2, image1, image2, points4D);

	transpose(points4D, points4DTransposed);

	cout << setprecision(17) << "4D Transposed Points are: \n" << points4DTransposed << endl;

	cout << endl;


	// create Mat object for triangulated 3D points
	Mat points3D;

	// printing triangulated 3D points into .txt file format
	ofstream csvTriangulation("../3D_Reconstruction_output/triangulated_3Dpoints.txt");

	print_points3D(csvTriangulation, points4DTransposed, points3D);

	csvTriangulation.close();


	// Intrinsic Camera matrix input
	// fx 0  cx 
	// 0  fy cy
	// 0  0   1

	ifstream csvIntrinsic("camera_intrinsic.txt");

	double arrayIntrinsic[3][3];

	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			csvIntrinsic >> arrayIntrinsic[row][col];

		}

	}

	Mat camera_matrix = Mat(3, 3, CV_64F, arrayIntrinsic);

	cout << "Camera Matrix " << endl << camera_matrix << endl;

	csvIntrinsic.close();

	cout << endl;


	// distortion coefficient input
	ifstream csvDist_coeffs("dist_coeffs.txt");

	double arrayDist_coeffs[5][1];

	for (int row = 0; row < 5; row++)
	{
		for (int col = 0; col < 1; col++)
		{
			csvDist_coeffs >> arrayDist_coeffs[row][col];

		}

	}

	Mat dist_coeffs = Mat(5, 1, CV_64F, arrayDist_coeffs);

	csvDist_coeffs.close();

	cout << "Distortion coefficients " << endl << dist_coeffs << endl;

	cout << endl;

	// from here, we can add as many images and movable points we want
	// in this example, we use 3 images and 8 movable points

	// 2D image_1 with new movable points input in pixel
	ifstream new_im1;

	new_im1.open("image1.txt");

	vector<Point2d> new_image1 = image_points(new_im1);

	new_im1.close();

	cout << " image points : " << endl << new_image1 << endl;

	cout << endl;

	// 2D image_2 with new movable points input in pixel
	ifstream new_im2;

	new_im2.open("image2.txt");

	vector<Point2d> new_image2 = image_points(new_im2);

	new_im2.close();

	cout << " image points : " << endl << new_image2 << endl;

	cout << endl;

	// 2D image_3 with new movable points input in pixel
	ifstream new_im3;

	new_im3.open("image3.txt");

	vector<Point2d> new_image3 = image_points(new_im3);

	new_im3.close();

	cout << " image points : " << endl << new_image3 << endl;

	cout << endl;


	//initial rotation input in radian
	ifstream csvInitialrotation("initial_rotation.txt");

	double arrayRot[1][3];

	for (int row = 0; row < 1; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			csvInitialrotation >> arrayRot[row][col];

		}

	}

	Mat rvec = Mat(1, 3, CV_64F, arrayRot);

	cout << "initial rotation: \n" << rvec << "\n" << endl;

	csvInitialrotation.close();

	cout << endl;


	//initial translation input in mm
	ifstream csvInitialtranslation("initial_translation.txt");

	double arrayTrans[1][3];

	for (int row = 0; row < 1; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			csvInitialtranslation >> arrayTrans[row][col];

		}

	}

	Mat tvec = Mat(1, 3, CV_64F, arrayTrans);

	cout << "initial translation: \n" << tvec << "\n" << endl;

	csvInitialtranslation.close();

	cout << endl;


	// function to estimate the pose of camera based on movable points
	// solvePnP iterative with Levenberg-Marquardt optimization is choosen as method to estimate the pose
	// initial rotation and translation is not used in this function
	Mat rvec_1, tvec_1,
		rvec_2, tvec_2,
		rvec_3, tvec_3;

	bool pose_new_image_1 = solvePnP(points3D, new_image1, camera_matrix, dist_coeffs, rvec_1, tvec_1, 0, SOLVEPNP_ITERATIVE);
	bool pose_new_image_2 = solvePnP(points3D, new_image2, camera_matrix, dist_coeffs, rvec_2, tvec_2, 0, SOLVEPNP_ITERATIVE);
	bool pose_new_image_3 = solvePnP(points3D, new_image3, camera_matrix, dist_coeffs, rvec_3, tvec_3, 0, SOLVEPNP_ITERATIVE);


	// printing rotation vector in degree
	ofstream csvRvecs_1("../3D_Reconstruction_output/Rotation_Vectors_1.txt");
	ofstream csvRvecs_2("../3D_Reconstruction_output/Rotation_Vectors_2.txt");
	ofstream csvRvecs_3("../3D_Reconstruction_output/Rotation_Vectors_3.txt");

	printRvecs(csvRvecs_1, rvec_1);
	printRvecs(csvRvecs_2, rvec_2);
	printRvecs(csvRvecs_3, rvec_3);

	csvRvecs_1.close();
	csvRvecs_2.close();
	csvRvecs_3.close();


	// printing translation in mm  with high precision decimal points
	ofstream csvTvecs_1("../3D_Reconstruction_output/Translation_Vectors_1.txt");
	ofstream csvTvecs_2("../3D_Reconstruction_output/Translation_Vectors_2.txt");
	ofstream csvTvecs_3("../3D_Reconstruction_output/Translation_Vectors_3.txt");

	printTvecs(csvTvecs_1, tvec_1);
	printTvecs(csvTvecs_2, tvec_2);
	printTvecs(csvTvecs_3, tvec_3);

	csvTvecs_1.close();
	csvTvecs_2.close();
	csvTvecs_3.close();


	// printing 3x3 rotations in rodrigues form 
	Mat R_1(3, 3, CV_64F);
	Mat R_2(3, 3, CV_64F);
	Mat R_3(3, 3, CV_64F);

	ofstream csvR_1("../3D_Reconstruction_output/Rotation3x3_1.txt");
	ofstream csvR_2("../3D_Reconstruction_output/Rotation3x3_2.txt");
	ofstream csvR_3("../3D_Reconstruction_output/Rotation3x3_3.txt");

	printRodriguesRotation(csvR_1, rvec_1, R_1);
	printRodriguesRotation(csvR_2, rvec_2, R_2);
	printRodriguesRotation(csvR_3, rvec_3, R_3);

	csvR_1.close();
	csvR_2.close();
	csvR_3.close();

	cout << endl;


	// create Mat object for extrinsic matrix and print it into .txt file format
	ofstream csvExtrinsicMatrix_1("../3D_Reconstruction_output/Extrinsic_Matrix_1.txt");
	ofstream csvExtrinsicMatrix_2("../3D_Reconstruction_output/Extrinsic_Matrix_2.txt");
	ofstream csvExtrinsicMatrix_3("../3D_Reconstruction_output/Extrinsic_Matrix_3.txt");

	Mat E_1 = extrinsic_matrix(R_1, tvec_1);
	Mat E_2 = extrinsic_matrix(R_2, tvec_2);
	Mat E_3 = extrinsic_matrix(R_3, tvec_3);

	print_extrinsic(csvExtrinsicMatrix_1, E_1);
	print_extrinsic(csvExtrinsicMatrix_2, E_2);
	print_extrinsic(csvExtrinsicMatrix_3, E_3);

	csvExtrinsicMatrix_1.close();
	csvExtrinsicMatrix_2.close();
	csvExtrinsicMatrix_3.close();

	cout << endl;


	// create Mat object for projection identity matrix
	Mat P_1 = projection_matrix(camera_matrix, E_1);
	Mat P_2 = projection_matrix(camera_matrix, E_2);
	Mat P_3 = projection_matrix(camera_matrix, E_3);

	// printing projection matrix in .txt file format
	ofstream csvProj_1("../3D_Reconstruction_output/Projection_Matrix_1.txt");
	ofstream csvProj_2("../3D_Reconstruction_output/Projection_Matrix_2.txt");
	ofstream csvProj_3("../3D_Reconstruction_output/Projection_Matrix_3.txt");

	print_projection_matrix(csvProj_1, P_1);
	print_projection_matrix(csvProj_2, P_2);
	print_projection_matrix(csvProj_3, P_3);

	csvProj_1.close();
	csvProj_2.close();
	csvProj_3.close();

	cout << endl;

	system("pause");

	return 0;

}


//2D image points extractor from text file data.
vector<Point2d> image_points(ifstream& csvImage)
{
	vector<Point2d> imagePoints;

	double u, v;

	while (!csvImage.eof())
	{
		csvImage >> u >> v;
		imagePoints.push_back(Point2d(u, v));
	}


	return imagePoints;
}


// printing triangulated 3D points into .txt file format
void print_points3D(ofstream& csvTriangulation, Mat& points4DTransposed, Mat& points3D)
{
	convertPointsFromHomogeneous(points4DTransposed, points3D);

	cout << setprecision(17) << "Reconstructed 3D Points are: \n" << points3D << "\n\n" << endl;

	csvTriangulation << setprecision(17) << points3D.at<double>(0, 0) << " " << points3D.at<double>(0, 1) << " " << points3D.at<double>(0, 2) << endl;
	csvTriangulation << setprecision(17) << points3D.at<double>(1, 0) << " " << points3D.at<double>(1, 1) << " " << points3D.at<double>(1, 2) << endl;
	csvTriangulation << setprecision(17) << points3D.at<double>(2, 0) << " " << points3D.at<double>(2, 1) << " " << points3D.at<double>(2, 2) << endl;
	csvTriangulation << setprecision(17) << points3D.at<double>(3, 0) << " " << points3D.at<double>(3, 1) << " " << points3D.at<double>(3, 2) << endl;
	csvTriangulation << setprecision(17) << points3D.at<double>(4, 0) << " " << points3D.at<double>(4, 1) << " " << points3D.at<double>(4, 2) << endl;
	csvTriangulation << setprecision(17) << points3D.at<double>(5, 0) << " " << points3D.at<double>(5, 1) << " " << points3D.at<double>(5, 2) << endl;
	csvTriangulation << setprecision(17) << points3D.at<double>(6, 0) << " " << points3D.at<double>(6, 1) << " " << points3D.at<double>(6, 2) << endl;
	csvTriangulation << setprecision(17) << points3D.at<double>(7, 0) << " " << points3D.at<double>(7, 1) << " " << points3D.at<double>(7, 2) << endl;
}

// printing rotation vectors into text file
void printRvecs(ofstream& csvRvecs, Mat& rvecs)
{
	double phi = 3.14159;

	cout << "Rotation Vector (in degree):  " << endl << setprecision(17) << (rvecs) * 180 / phi << "\n\n" << endl;

	csvRvecs << setprecision(17) << rvecs.at<double>(0) * 180 / phi << " " << rvecs.at<double>(1) * 180 / phi << " " << rvecs.at<double>(2) * 180 / phi << endl;
}


// printing translation vectors into text file
void printTvecs(ofstream& csvTvecs, Mat& tvecs)
{
	cout << "Translation Vector (in mm):" << endl << setprecision(17) << tvecs << "\n\n" << endl;

	csvTvecs << setprecision(17) << tvecs.at<double>(0) << " " << tvecs.at<double>(1) << " " << tvecs.at<double>(2) << endl;
}


// printing 3x3 rotation matrix in rodrigues form into text file
void printRodriguesRotation(ofstream& csvR, Mat& rvec, Mat& R)
{

	Rodrigues(rvec, R);

	cout << "The 3x3 rotation matrix is: \n" << setprecision(17) << R << "\n\n" << endl;

	csvR << setprecision(17) << R.at<double>(0, 0) << " " << R.at<double>(0, 1) << " " << R.at<double>(0, 2) << endl;
	csvR << setprecision(17) << R.at<double>(1, 0) << " " << R.at<double>(1, 1) << " " << R.at<double>(1, 2) << endl;
	csvR << setprecision(17) << R.at<double>(2, 0) << " " << R.at<double>(2, 1) << " " << R.at<double>(2, 2) << endl;

}


// create mat object for extrinsic matrix
Mat extrinsic_matrix(Mat& R, Mat& tvec)
{
	Mat E = (Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tvec.at<double>(0),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tvec.at<double>(1),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tvec.at<double>(2),
		0, 0, 0, 1);

	return E;
}


// printing extrinsic matrix into text file
void print_extrinsic(ofstream& csvExtrinsicMatrix, Mat& E)
{
	cout << "Extrinsic matrix is:  \n" << setprecision(17) << E << "\n\n" << endl;

	csvExtrinsicMatrix << setprecision(17) << E.at<double>(0, 0) << " " << E.at<double>(0, 1) << " " << E.at<double>(0, 2) << " " << E.at<double>(0, 3) << endl;
	csvExtrinsicMatrix << setprecision(17) << E.at<double>(1, 0) << " " << E.at<double>(1, 1) << " " << E.at<double>(1, 2) << " " << E.at<double>(1, 3) << endl;
	csvExtrinsicMatrix << setprecision(17) << E.at<double>(2, 0) << " " << E.at<double>(2, 1) << " " << E.at<double>(2, 2) << " " << E.at<double>(2, 3) << endl;
	csvExtrinsicMatrix << setprecision(17) << E.at<double>(3, 0) << " " << E.at<double>(3, 1) << " " << E.at<double>(3, 2) << " " << E.at<double>(3, 3) << endl;
}


// create mat object for projection matrix
Mat projection_matrix(Mat& camera_matrix, Mat& extrinstic_matrix)
{
	Mat ProjIdentity = (Mat_<double>(3, 4) << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);

	Mat P = camera_matrix * ProjIdentity * extrinstic_matrix;

	return P;
}


// printing projection matrix into text file
void print_projection_matrix(ofstream& csvProj, Mat& P)
{
	cout << "The projection matrix is: \n" << setprecision(17) << P << "\n\n" << endl;

	csvProj << setprecision(17) << P.at<double>(0, 0) << " " << P.at<double>(0, 1) << " " << P.at<double>(0, 2) << " " << P.at<double>(0, 3) << endl;
	csvProj << setprecision(17) << P.at<double>(1, 0) << " " << P.at<double>(1, 1) << " " << P.at<double>(1, 2) << " " << P.at<double>(1, 3) << endl;
	csvProj << setprecision(17) << P.at<double>(2, 0) << " " << P.at<double>(2, 1) << " " << P.at<double>(2, 2) << " " << P.at<double>(2, 3) << endl;
}