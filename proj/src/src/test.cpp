/*
 * proj.cpp
 * MS403 Project code1
 */

#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <stdio.h>
#include <iostream>
#include <time.h>

#define pi 3.1415926535

using namespace std;
using namespace Eigen;

int main()
{
    MatrixXd a(3, 3);
    a << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    MatrixXd b = a.block(0, 0, 1, 3)*a.block(0, 0, 1, 3).transpose();
    double c = b(0, 0);
    cout << c;
    return 0;
}