#include <iostream>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include <math.h>

using namespace std;
using namespace Eigen;

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
    cout << "Euler2Quaternion result is:" << endl;
    cout << "x = " << q.x() << endl;
    cout << "y = " << q.y() << endl;
    cout << "z = " << q.z() << endl;
    cout << "w = " << q.w() << endl << endl;
    return q;
}
Eigen::Vector3d Quaterniond2Euler(const double x, const double y, const double z, const double w)
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
Eigen::Matrix3d Quaternion2RotationMatrix(const double x, const double y, const double z, const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();

    cout << "Quaternion2RotationMatrix result is:" << endl;
    cout << "R = " << endl << R << endl << endl;
    return R;
}
Eigen::Quaterniond rotationMatrix2Quaterniond(Eigen::Matrix3d R)
{
    Eigen::Quaterniond q = Eigen::Quaterniond(R);
    q.normalize();
    cout << "RotationMatrix2Quaterniond result is:" << endl;
    cout << "x = " << q.x() << endl;
    cout << "y = " << q.y() << endl;
    cout << "z = " << q.z() << endl;
    cout << "w = " << q.w() << endl << endl;
    return q;
}
Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3d R = q.matrix();
    cout << "Euler2RotationMatrix result is:" << endl;
    cout << "R = " << endl << R << endl << endl;
    return R;
}
Eigen::Vector3d RotationMatrix2euler(Eigen::Matrix3d R)
{
    Eigen::Matrix3d m;
    m = R;
    Eigen::Vector3d euler = m.eulerAngles(0, 1, 2);
    cout << "RotationMatrix2euler result is:" << endl;
    cout << "x = " << euler[2] << endl;
    cout << "y = " << euler[1] << endl;
    cout << "z = " << euler[0] << endl << endl;
    return euler;
}
int main(int argc, char** argv)
{
    //this is euler2Quaternion transform function,please input your euler angle//欧拉角转换到四元数//
    euler2Quaternion(-3.01233, -0.0675655, 2.11925);

    //this is Quaternion2Euler transform function,please input your Quaternion angle//四元数转换为欧拉//
    Quaterniond2Euler(0.557533, -0.569317, -0.598503, 0.0826407);

    //this is Quaternion2RotationMatrix transform function,please input your Quaternion parameter//四元数转旋转矩阵//
    Quaternion2RotationMatrix(0.961151, -0.0883379, -0.221842, -0.138459);//10711ms
    //Quaternion2RotationMatrix(0.932656, 0.31192, -0.00949, -0.18102);//13300ms
    Quaternion2RotationMatrix(0.934572, 0.934872, 0.121798, -0.30821);//232651,11000ms

    //this is rotationMatrix2Quaterniond transform function,please input your RotationMatrix parameter like following//旋转矩阵转化为四元数//
    //Eigen::Vector3d x_axiz, y_axiz, z_axiz;
    //x_axiz << 0.6530031635738726, 0.754712416766035, 0.06321420996646843;
    //y_axiz << 0.1680070647414041, -0.06296533768625048, -0.9837728357944399;
    //z_axiz << -0.7384852703740892, 0.6530272078782223, -0.1679135825693716;
    //Eigen::Matrix3d R;
    //R << x_axiz, y_axiz, z_axiz;
    //rotationMatrix2Quaterniond(R);

    //this is euler2RotationMatrix transform function,please input your euler angle for the function parameter//欧拉角转换为旋转矩阵//
    euler2RotationMatrix(5.98406, 1.5451, 3.16652);//10700ms
    //euler2RotationMatrix(5.03644, 1.47147, 3.19145);//13305ms
    euler2RotationMatrix(6.15817, 1.02201, 3.06643);//232651,11013ms


    //this is RotationMatrix2euler transform function,please input your RotationMatrix for the function parameter//旋转矩阵转换为欧拉角//
    Eigen::Vector3d x_axiz, y_axiz, z_axiz;
    x_axiz << 0.6530031635738726, 0.754712416766035, 0.06321420996646843;
    y_axiz << 0.1680070647414041, -0.06296533768625048, -0.9837728357944399;
    z_axiz << -0.7384852703740892, 0.6530272078782223, -0.1679135825693716;
    Eigen::Matrix3d R;
    RotationMatrix2euler(R);
    cout << "All transform is done!" << endl;
}

//x_axiz << 0.2699300039396583, 0.7736254137571271, 0.5732726333623908;
//y_axiz << 0.5893258112741561, 0.3380972463121301, -0.7337474635064933;
//z_axiz << -0.761467583774381, 0.5359048154525847, -0.3646548335557027;