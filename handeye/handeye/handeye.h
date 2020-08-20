/*!
 * \file handeye.h
 * Hand-Eye Calibration by Different Methods.
 */

#pragma once

#include <opencv2\opencv.hpp>
#include <vector>
#include "quaternion.h"

using namespace cv;
using namespace std;



/**
 * Compute the skew symmetric matrix.
 * as defined  [e]x  =    [ 0,-e3,e2;
 *                          e3,0,-e1;
 *                          -e2,e1,0 ]
 *
 * @Returns  cv::Mat 3x3 matrix
 * @param A [in] 3x1 vector
 */
Mat skew(Mat A)
{
	CV_Assert(A.cols == 1 && A.rows == 3);
	Mat B(3, 3, CV_64FC1);

	B.at<double>(0, 0) = 0.0;
	B.at<double>(0, 1) = -A.at<double>(2, 0);
	B.at<double>(0, 2) = A.at<double>(1, 0);

	B.at<double>(1, 0) = A.at<double>(2, 0);
	B.at<double>(1, 1) = 0.0;
	B.at<double>(1, 2) = -A.at<double>(0, 0);

	B.at<double>(2, 0) = -A.at<double>(1, 0);
	B.at<double>(2, 1) = A.at<double>(0, 0);
	B.at<double>(2, 2) = 0.0;

	return B;
}


/**
 * Creates a dual quaternion from a rotation matrix and a translation vector.
 *
 * @Returns  void
 * @param q [out] q
 * @param qprime [out] q'
 * @param R [in] Rotation
 * @param t [in] Translation
 */
void getDualQ(Mat q, Mat qprime, Mat R, Mat t)
{
	Mat r(3, 1, CV_64FC1);
	Mat l(3, 1, CV_64FC1);
	double theta;
	Mat tempd(1, 1, CV_64FC1);
	double d;
	Mat c(3, 1, CV_64FC1);
	Mat m(3, 1, CV_64FC1);
	Mat templ(3, 1, CV_64FC1);
	Mat tempqt(1, 1, CV_64FC1);
	double qt;
	Mat tempml(3, 1, CV_64FC1);

	Rodrigues(R, r);
	theta = norm(r);
	l = r / theta;
	tempd = l.t() * t;
	d = tempd.at<double>(0, 0);

	c = 0.5 * (t - d * l) + cos(theta / 2) / sin(theta / 2) * l.cross(t);
	m = c.cross(l);

	q.at<double>(0, 0) = cos(theta / 2);
	templ = sin(theta / 2) * l;
	templ.copyTo(q(Rect(0, 1, 1, 3)));

	tempqt = -0.5 * templ.t() * t;
	qt = tempqt.at<double>(0, 0);
	tempml = 0.5 * (q.at<double>(0, 0) * t + t.cross(templ));

	qprime.at<double>(0, 0) = qt;
	tempml.copyTo(qprime(Rect(0, 1, 1, 3)));

}

/**
 * Compute the Kronecker tensor product of matrix A and B.
 *
 * @Returns  cv::Mat (MP)x(NQ) matrix
 * @param A [in] MxN matrix
 * @param B [in] PxQ matrix
 */
Mat kron(Mat A, Mat B)
{
	Mat C(A.rows * B.rows, A.cols * B.cols, CV_64FC1, Scalar(0));

	for (int i = 0; i < A.rows; i++)
		for (int j = 0; j < A.cols; j++)
			C(Rect(B.cols * j, B.rows * i, B.cols, B.rows)) = A.at<double>(i, j) * B;

	return C;
}

/**
 * Signum function.
 * For each element of X, SIGN(X) returns 1 if the element is greater than zero,
 * return 0 if it equals zero and -1 if it is less than zero.
 *
 * @Returns  double
 * @param a [in]
 */
double sign(double a)
{
	if (a > 0)
		return 1;
	else if (a < 0)
		return -1;
	else
		return 0;
}

/**
 * Hand/Eye calibration using Tsai' method.
 * Read the paper "A New Technique for Fully Autonomous and Efficient 3D Robotics
 * Hand-Eye Calibration, Tsai, 1989" for further details.
 *
 * Solving AX=XB
 * @Returns  void
 * @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
 * @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
 * @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
 */
void Tsai_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nStatus = Hgij.size();

	Mat Rgij(3, 3, CV_64FC1);
	Mat Rcij(3, 3, CV_64FC1);

	Mat rgij(3, 1, CV_64FC1);
	Mat rcij(3, 1, CV_64FC1);

	double theta_gij;
	double theta_cij;

	Mat rngij(3, 1, CV_64FC1);
	Mat rncij(3, 1, CV_64FC1);

	Mat Pgij(3, 1, CV_64FC1);
	Mat Pcij(3, 1, CV_64FC1);

	Mat tempA(3, 3, CV_64FC1);
	Mat tempb(3, 1, CV_64FC1);

	Mat A;
	Mat b;
	Mat pinA;

	Mat Pcg_prime(3, 1, CV_64FC1);
	Mat Pcg(3, 1, CV_64FC1);
	Mat PcgTrs(1, 3, CV_64FC1);

	Mat Rcg(3, 3, CV_64FC1);
	Mat eyeM = Mat::eye(3, 3, CV_64FC1);

	Mat Tgij(3, 1, CV_64FC1);
	Mat Tcij(3, 1, CV_64FC1);

	Mat tempAA(3, 3, CV_64FC1);
	Mat tempbb(3, 1, CV_64FC1);

	Mat AA;
	Mat bb;
	Mat pinAA;

	Mat Tcg(3, 1, CV_64FC1);

	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);

		Rodrigues(Rgij, rgij);
		Rodrigues(Rcij, rcij);

		theta_gij = norm(rgij);
		theta_cij = norm(rcij);

		rngij = rgij / theta_gij;
		rncij = rcij / theta_cij;

		Pgij = 2 * sin(theta_gij / 2) * rngij;
		Pcij = 2 * sin(theta_cij / 2) * rncij;

		tempA = skew(Pgij + Pcij);
		tempb = Pcij - Pgij;

		A.push_back(tempA);
		b.push_back(tempb);
	}

	//Compute rotation
	invert(A, pinA, DECOMP_SVD);

	Pcg_prime = pinA * b;
	Pcg = 2 * Pcg_prime / sqrt(1 + norm(Pcg_prime) * norm(Pcg_prime));
	PcgTrs = Pcg.t();
	Rcg = (1 - norm(Pcg) * norm(Pcg) / 2) * eyeM + 0.5 * (Pcg * PcgTrs + sqrt(4 - norm(Pcg) * norm(Pcg)) * skew(Pcg));

	//Computer Translation 
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

		tempAA = Rgij - eyeM;
		tempbb = Rcg * Tcij - Tgij;

		AA.push_back(tempAA);
		bb.push_back(tempbb);
	}

	invert(AA, pinAA, DECOMP_SVD);
	Tcg = pinAA * bb;

	Rcg.copyTo(Hcg(Rect(0, 0, 3, 3)));
	Tcg.copyTo(Hcg(Rect(3, 0, 1, 3)));
	Hcg.at<double>(3, 0) = 0.0;
	Hcg.at<double>(3, 1) = 0.0;
	Hcg.at<double>(3, 2) = 0.0;
	Hcg.at<double>(3, 3) = 1.0;

}
