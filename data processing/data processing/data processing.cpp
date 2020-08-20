// data processing.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "quaternion.h"
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <math.h>


using namespace Eigen;
using namespace std;
//删除字符串中空格，制表符tab等无效字符
string Trim(string& str)
{
	//str.find_first_not_of(" \t\r\n"),在字符串str中从索引0开始，返回首次不匹配"\t\r\n"的位置
	str.erase(0, str.find_first_not_of(" \t\r\n"));
	str.erase(str.find_last_not_of(" \t\r\n") + 1);
	return str;
}
//quaternion transform to Euler
Eigen::Vector3d Quaterniond2Euler( const double x, const double y, const double z, const double w)
{
	Eigen::Quaterniond q;
	q.x() = x;
	q.y() = y;
	q.z() = z;
	q.w() = w;
	Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
	cout << "Quaterniond2Euler result is:" << endl;
	cout << "x = " << euler[2] << endl;
	cout << "y = " << euler[1] << endl;
	cout << "z = " << euler[0] << endl << endl;
	return euler;
}

int main()
{
	ifstream fin("ar_2020216_232651.csv"); //open file
	string line;
	ofstream outFile;
	outFile.open("euler_ar_2020216_232651.csv", ios::out); //Store files

	Mat q1(4, 1, CV_16FC1);//X in AX=XB
	Mat q2(4, 1, CV_16FC1);//B in AX=XB,the data of AR(quaternion)

	//Use the transform function to convert the rotation matrix to quaternion
	q1 = (Mat_<double>(4, 1) << 0.596264, -0.686273, -0.336134, -0.245992);
	//q1 = (Mat_<double>(4, 1) <<0.557533, -0.569317, -0.598503, 0.0826407);
	//q1 = (Mat_<double>(4, 1) << 0.0609748, 0.868787, 0.0727871, -0.485997);

	//Read the file and assign it to the variable
	while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
	{
		istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
		vector<string> fields; //声明一个字符串向量
		string field;
		while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		{
			fields.push_back(field); //将刚刚读取的字符串添加到向量fields中
		}
		string time = Trim(fields[0]); //清除掉向量fields中第一个元素的无效字符，并赋值给变量time
		string w = Trim(fields[1]); //清除掉向量fields中第二个元素的无效字符，并赋值给变量w
		string x = Trim(fields[2]); //清除掉向量fields中第三个元素的无效字符，并赋值给变量x
		string y = Trim(fields[3]); //清除掉向量fields中第三个元素的无效字符，并赋值给变量y
		string z = Trim(fields[4]); //清除掉向量fields中第三个元素的无效字符，并赋值给变量z


		q2 = (Mat_<double>(4, 1) << stod(w) , stod(x), stod(y), stod(z));

		//cout << "q2" << q2 << "\t" << endl;
			//<< time << "\t" << w << "\t" << x << "\t" << y << "\t" << z << endl;

		//qin is XB
		Mat	qin = qmult(q1, q2);

		//qout is A=X*b*X^-1
		Mat qout = qnorm(qmult(qin, qconj(qnorm(q1))));
		
		double qout_w = qout.at<double>(0, 0);
		double qout_x = qout.at<double>(1, 0);
		double qout_y = qout.at<double>(2, 0);
		double qout_z = qout.at<double>(3, 0);

		//Use the transform function to convert the  quaternion to Euler angle
		Eigen::Vector3d euler_out = Quaterniond2Euler(qout_x, qout_y, qout_z, qout_w);

		outFile << time << ',' << euler_out[2] << ',' << euler_out[1] << ',' << euler_out[0] << endl;

	}
	outFile.close();
	return 0;
}
