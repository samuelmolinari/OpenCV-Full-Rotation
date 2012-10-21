#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <map>
#include <math.h>

using namespace cv;
using namespace std;

struct Equ {
    double m;
    double c;
    bool isVertical;
    bool isHorizontal;
};

class Utils
{
public:
    
    
    
    Utils();
    static double degreeToRadian(double degree);
    static double radianToDegree(double radian);
    static double round(double d);
    static void rotate(Mat &src, Mat &dst, double rotation);
    static Size rotationNewCanvasSize(double degree,double angle,double h);
    static map<string,int> rotationExtraMargins(Size &original, Size &newSize);
    static map<string,Point> getCorners(Size &original,map<string,int> &margins);
    static map<string,Point> getProjectedCorners(Size &s,double h,double degree,double angle);
    static Point getCentreBetweenPoints(Point &a, Point &b);
    static map<string,Point> getCentreBetweenOriginalsAndProjections(map<string,Point> &originals,map<string,Point> &projections);
    static Equ getLinearEquation(Point &a,Point &b);
    static Equ getPerpendicular(Equ e,Point p);
    static map<string,Equ> getLinearEquationBetweenOriginalsAndProjections(map<string,Point> &originals,map<string,Point> &projections);
    static map<string,Equ> getPerpendicularLinearEquation(map<string,Point> &originals,map<string,Point> &projections,map<string,Point> &centre);
    static Point getColisionPoint(Equ e1,Equ e2);
    static double solveEquationY(Equ e,double x);
    static double solveEquationX(Equ e,double y);
    
};

#endif // UTILS_H
