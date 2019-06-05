/* This program is intended to reconstruct 3D object points based on perspective projection concept
 * reconstruction is based on available projection matrix from control points
 * additional object points (movable) can be added as much as you want and reconstructed  based on projection matrix from control points
 * number of images and its projection matrices needed are 2 and 2 respectively
 * All input data is in .txt file format
*/
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	// 2d image points 1
	// taken at distance 100 mm to the right (positive) of camera coordinate in X direction
	ifstream csvImage1("single_X100.txt");

	vector<Point2d> image_points1;

	double u, v;

	while (!csvImage1.eof())
	{
		csvImage1 >> u >> v;
		image_points1.push_back(Point2d(u, v));
	}

	cout << " image points 1 : " << endl << image_points1 << endl;

	csvImage1.close();

	cout << endl;

	// 2d image points 2
	// taken at distance 100 mm to the left (negative) of camera coordinate in X direction
	ifstream csvImage2("single_X-100.txt");

	vector<Point2d> image_points2;

	double a, b;

	while (!csvImage2.eof())
	{
		csvImage2 >> a >> b;
		image_points2.push_back(Point2d(a, b));
	}

	cout << " image points 2 : " << endl << image_points2 << endl;

	csvImage2.close();

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


	// outputing triangulation
	ofstream csvTriangulation("../3D_Reconstruction_output/triangulated_3Dpoints.txt");

	// solve triangulation
	Mat points4D;

	Mat points4DTransposed;

	triangulatePoints(proj1, proj2, image_points1, image_points2, points4D);

	transpose(points4D, points4DTransposed);

	cout << setprecision(17) << "4D Transposed Points are: \n" << points4DTransposed << endl;

	cout << endl;

	// convert triangulated result from homogenous 4D coordinates to euclidean 3D coordinates
	// feel free to adjust how many object points are getting outputted here
	// number of output should be the same as number of inputted image points
	Mat points3D;

	convertPointsFromHomogeneous(points4DTransposed, points3D);

	cout << setprecision(17) << "Reconstructed 3D Points are: \n" << points3D << endl;

	csvTriangulation << setprecision(17) << points3D.at<double>(0, 0) << " " << points3D.at<double>(0, 1) << " " << points3D.at<double>(0, 2) << endl;
	csvTriangulation << setprecision(17) << points3D.at<double>(1, 0) << " " << points3D.at<double>(1, 1) << " " << points3D.at<double>(1, 2) << endl;
	csvTriangulation << setprecision(17) << points3D.at<double>(2, 0) << " " << points3D.at<double>(2, 1) << " " << points3D.at<double>(2, 2) << endl;
	csvTriangulation << setprecision(17) << points3D.at<double>(3, 0) << " " << points3D.at<double>(3, 1) << " " << points3D.at<double>(3, 2) << endl;
	csvTriangulation << setprecision(17) << points3D.at<double>(4, 0) << " " << points3D.at<double>(4, 1) << " " << points3D.at<double>(4, 2) << endl;
	csvTriangulation << setprecision(17) << points3D.at<double>(5, 0) << " " << points3D.at<double>(5, 1) << " " << points3D.at<double>(5, 2) << endl;
	csvTriangulation << setprecision(17) << points3D.at<double>(6, 0) << " " << points3D.at<double>(6, 1) << " " << points3D.at<double>(6, 2) << endl;
	csvTriangulation << setprecision(17) << points3D.at<double>(7, 0) << " " << points3D.at<double>(7, 1) << " " << points3D.at<double>(7, 2) << endl;

	csvTriangulation.close();

	system("pause");

	return 0;

}