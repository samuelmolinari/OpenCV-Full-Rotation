/**
 * Tools used within the namespace drlib
 *
 * @author Samuel Molinari
 * @version 14/04/2012
 */

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <map>
#include <math.h>

#ifndef UTILS_H
#define UTILS_H

using namespace cv;
using namespace std;

struct Equ {
    double m;
    double c;
    bool isVertical;
    bool isHorizontal;
};

namespace drlib {

    class Utils {
    public:
        Utils();
        
        static double degreeToRadian(double degree);
        static double radianToDegree(double radian);
        static double round(double d);
        static void rotate(cv::Mat &src, cv::Mat &dst, double degree);
        static void resize(cv::Mat &src, cv::Mat &dst,const cv::Size &size, bool keepRatio=false);
        static void setupForRecognition(cv::Mat &src, cv::Mat &dst);
        static cv::Point rectCentre(const cv::Rect &rect);
        static bool pointInRect(const cv::Rect &r,const cv::Point &p);
        static cv::Point positionRefToCentre(const cv::Point &p,const cv::Size &s);
        static cv::Point positionRefToTopLeft(const cv::Point &p,const cv::Size &s);
        static cv::Point getNormalPosition(const cv::Rect r,const cv::Size &s,const cv::Size &sd,double angle);
        
    private:
        static cv::Size _rotate_newCanvasSize(double degree,double angle,double h);
        static std::map<std::string,int> _rotate_extraMargins(cv::Size &original,cv::Size &newSize);
        static std::map<std::string,cv::Point> _rotate_getCorners(cv::Size &original,std::map<std::string,int> &margins);
        static std::map<std::string,cv::Point> _rotate_getProjectedCorners(cv::Size &s,double h,double degree,double angle);
        static cv::Point _rotate_getCentreBetweenPoints(cv::Point &a, cv::Point &b);
        static std::map<string,cv::Point> _rotate_getCentreBetweenOriginalsAndProjections(std::map<std::string,cv::Point> &originals,map<string,cv::Point> &projections);
        static Equ _rotate_getLinearEquation(cv::Point &a,cv::Point &b);
        static Equ _rotate_getPerpendicular(Equ e,cv::Point p);
        static std::map<std::string,Equ> _rotate_getLinearEquationBetweenOriginalsAndProjections(std::map<std::string,cv::Point> &originals,std::map<std::string,cv::Point> &projections);
        static std::map<std::string,Equ> _rotate_getPerpendicularLinearEquation(std::map<std::string,cv::Point> &originals,std::map<std::string,cv::Point> &projections,std::map<std::string,cv::Point> &centre);
        static cv::Point _rotate_getColisionPoint(Equ e1,Equ e2);
        static double _rotate_solveEquationY(Equ e,double x);
        static double _rotate_solveEquationX(Equ e,double y);
        
    };
}

#endif // UTILS_H
