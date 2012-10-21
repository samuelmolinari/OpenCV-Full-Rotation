#define NDEBUG

/**
 * 
 * @author Samuel Molinari
 * @version 14/04/2012
 */

#include "utils.h"

using namespace drlib;

Utils::Utils() {
}

double Utils::round(double number) {
    return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
}

double Utils::degreeToRadian(double degree) {
    return (M_PI*degree)/180.0;
}

double Utils::radianToDegree(double radian) {
    return (180*radian)/M_PI;
}

void Utils::resize(cv::Mat &src,cv::Mat &dst,const cv::Size &s,bool keepRatio){
    float ratio = 0.0;
    cv::Size size = s;
    
    if(keepRatio) {
        if(src.cols > size.width || src.rows > size.height) {
            if(src.rows > src.cols) {
                ratio = (float)size.height/(float)src.rows;
                size = cv::Size(src.cols*ratio,size.height);
            } else {
                ratio = (float)size.width/(float)src.cols;
                size = cv::Size(size.width,src.rows*ratio);
            }
            cv::resize(src,dst,size);
        }
    } else {
        cv::resize(src,dst,size);
    }
}

void Utils::setupForRecognition(cv::Mat &src, cv::Mat &dst) {
    dst = src;
    resize(dst,dst,Size(120,120));
    dst = dst(Rect(20,10,80,100));
    resize(dst,dst,Size(120,120));
}

Point Utils::rectCentre(const cv::Rect &rect) {
    return Point(rect.width/2.0+rect.x,rect.height/2.0+rect.y);
}

bool Utils::pointInRect(const cv::Rect &r,const cv::Point &p) {
    return p.x >= r.x && p.x <= r.x+r.width && p.y >= r.y && p.y <= r.y+r.height;
}

cv::Point Utils::positionRefToCentre(const cv::Point &p,const cv::Size &s) {
    double w = s.width/2.0;
    double h = s.height/2.0;
    
    return cv::Point(p.x-w,h-p.y);
}

cv::Point Utils::positionRefToTopLeft(const cv::Point &p,const cv::Size &s) {
    double w = s.width/2.0;
    double h = s.height/2.0;
    
    return cv::Point(p.x+w,h-p.y);
}

cv::Point Utils::getNormalPosition(const cv::Rect r,const cv::Size &s,const cv::Size &sd,double angle) {
    
    cv::Point p = rectCentre(r);
    p = positionRefToCentre(p,s);
    
    double h = sqrt(pow(p.x,2)+pow(p.y,2));

    double add = 0;
    int sign = 1;
    double radian;
    
    if((p.x<0 && p.y<0)) {
        add = M_PI;
        sign = 1;
    } else if(p.x<0) {
        add = M_PI;
        sign = -1;
    } else if(p.y<0) {
        add = M_PI*2;
        sign = -1;
    }
    
    if(p.x != 0) {
        radian = add+atan(p.y/(double)p.x);
    } else {
        radian = add+sign*asin(p.y/h);
    }
    
    double targetRadian = radian-angle;
    
    p = cv::Point(h*cos(targetRadian),h*sin(targetRadian));
    
    return positionRefToTopLeft(p,sd);
}

cv::Size Utils::_rotate_newCanvasSize(double degree,double angle,double h) {
    cv::Size canvas(0,0);
    double radian = degreeToRadian(degree);
    if((degree >= 0 && degree <= 90) || (degree >= 181 && degree <= 270)) {
        canvas.height = round((h*fabs(sin(angle+radian)))*2);
        canvas.width = round((h*fabs(cos(M_PI-angle+radian)))*2);
    } else {
        canvas.height = round((h*fabs(sin(M_PI-angle+radian)))*2);
        canvas.width = round((h*fabs(cos(angle+radian)))*2);
    }
    return canvas;
}

double Utils::_rotate_solveEquationY(Equ e,double x) {
    
    cout << x << endl;
    
    if(e.isVertical) return x;
    
    return e.m*x+e.c;
}

double Utils::_rotate_solveEquationX(Equ e, double y) {
    
    if(e.isVertical) return e.c;
    if(e.isHorizontal) return 0;

    return (y-e.c)/e.m;
}

std::map<std::string,int> Utils::_rotate_extraMargins(cv::Size &original, cv::Size &newSize) {
    
    std::map<std::string,int> m = std::map<std::string,int>();
    
    if(newSize.height >= original.height) {
        m["top"] = round((newSize.height-original.height)/2.0);
        m["bottom"] = round((newSize.height-original.height)/2.0);
    } else {
        m["top"] = 0;
        m["bottom"] = 0;
    }
    
    if(newSize.width >= original.width) {
        m["left"] = round((newSize.width-original.width)/2.0);
        m["right"] = round((newSize.width-original.width)/2.0);
    } else {
        m["left"] = 0;
        m["right"] = 0;
    }
    
    return m;
}

std::map<std::string,cv::Point> Utils::_rotate_getCorners(cv::Size &original,std::map<std::string,int> &margins) {
    std::map<std::string,cv::Point> m = std::map<std::string,cv::Point>();
    
    m["tl"] = cv::Point(margins["left"],margins["top"]);
    m["tr"] = cv::Point(margins["left"]+original.width,margins["top"]);
    m["bl"] = cv::Point(margins["left"],margins["top"]+original.height);
    m["br"] = cv::Point(margins["left"]+original.width,margins["top"]+original.height);
    
    return m;
    
}

std::map<std::string,cv::Point> Utils::_rotate_getProjectedCorners(cv::Size &s,double h,double degree,double angle) {
    
    std::map<std::string,cv::Point> m = std::map<std::string,cv::Point>();
    double radian = degreeToRadian(degree);
    int top,left;
    
    m["tl"] = cv::Point(
                round(s.width/2.0+h*cos(M_PI-angle+radian)),
                round(s.height/2.0-h*sin(M_PI-angle+radian)));
    
    top = m["tl"].y;
    left = m["tl"].x;
    
    m["tr"] = cv::Point(
                round(s.width/2.0+h*cos(angle+radian)),
                round(s.height/2.0-h*sin(angle+radian)));
    
    if(top>m["tr"].y) top=m["tr"].y;
    if(left>m["tr"].x) left=m["tr"].x;
    
    m["br"] = cv::Point(
                round(s.width/2.0+h*cos(-angle+radian)),
                round(s.height/2.0-h*sin(-angle+radian)));
    
    if(top>m["br"].y) top=m["br"].y;
    if(left>m["br"].x) left=m["br"].x;
    
    m["bl"] = cv::Point(
                round(s.width/2.0+h*cos(M_PI+angle+radian)),
                round(s.height/2.0-h*sin(M_PI+angle+radian)));
    
    if(top>m["bl"].y) top=m["bl"].y;
    if(left>m["bl"].x) left=m["bl"].x;
    
    m["tl"].x -= left;
    m["tr"].x -= left;
    m["bl"].x -= left;
    m["br"].x -= left;
    
    m["tl"].y -= top;
    m["tr"].y -= top;
    m["bl"].y -= top;
    m["br"].y -= top;
    
    
    return m;
    
}

cv::Point Utils::_rotate_getCentreBetweenPoints(cv::Point &a, cv::Point &b) {
    cv::Point c(0,0);
    double tmp;
    
    tmp = abs(a.x-b.x)/2.0;
    
    if(b.x <= a.x) c.x = round(tmp+b.x);
    else c.x = round(tmp+a.x);
    
    tmp = abs(a.y-b.y)/2.0;
    
    if(b.y <= a.y) c.y = round(tmp+b.y);
    else c.y = round(tmp+a.y);
    
    return c;
}

std::map<std::string,cv::Point> Utils::_rotate_getCentreBetweenOriginalsAndProjections(std::map<std::string,cv::Point> &originals,std::map<std::string,cv::Point> &projections) {
    std::map<std::string,cv::Point> m = std::map<std::string,cv::Point>();
    
    m["tl"] = _rotate_getCentreBetweenPoints(originals["tl"],projections["tl"]);
    m["tr"] = _rotate_getCentreBetweenPoints(originals["tr"],projections["tr"]);
    m["bl"] = _rotate_getCentreBetweenPoints(originals["bl"],projections["bl"]);
    m["br"] = _rotate_getCentreBetweenPoints(originals["br"],projections["br"]);
    
    return m;
    
}

Equ Utils::_rotate_getLinearEquation(cv::Point &a,cv::Point &b) {
    
    Equ equation;
    
    equation.isVertical = false;
    equation.isHorizontal = false;
    
    if(a.x-b.x==0) {
        equation.c = a.x;
        equation.m = 0;
        equation.isVertical = true;
    } else if(a.y-b.y==0) {
        equation.c = a.y;
        equation.m = 0;
        equation.isHorizontal = true;
    } else {
        equation.m = (double)(a.y-b.y)/(a.x-b.x);
        equation.c = a.y-equation.m*a.x;
    }
    
    return equation;
    
}

Equ Utils::_rotate_getPerpendicular(Equ e,cv::Point p) {
    Equ equation;
    
    equation.isVertical = false;
    equation.isHorizontal = false;
    
    if(e.isHorizontal) {
        equation.c = p.x;
        equation.m = 0;
        equation.isVertical = true;
    } else if(e.isVertical) {
        equation.c = p.y;
        equation.m = 0;
        equation.isHorizontal = true;        
    } else {
        equation.m = -1.0/e.m;
        equation.c = -p.x*equation.m+p.y;
    }
    
    return equation;
    
}

std::map<std::string,Equ> Utils::_rotate_getLinearEquationBetweenOriginalsAndProjections(std::map<std::string,cv::Point> &originals,std::map<std::string,cv::Point> &projections) {
    std::map<std::string,Equ> m = map<std::string,Equ>();
    
    m["tl"] = _rotate_getLinearEquation(originals["tl"],projections["tl"]);
    m["tr"] = _rotate_getLinearEquation(originals["tr"],projections["tr"]);
    m["bl"] = _rotate_getLinearEquation(originals["bl"],projections["bl"]);
    m["br"] = _rotate_getLinearEquation(originals["br"],projections["br"]);
    
    return m;
}

std::map<std::string,Equ> Utils::_rotate_getPerpendicularLinearEquation(std::map<std::string,cv::Point> &originals,std::map<std::string,cv::Point> &projections,std::map<std::string,cv::Point> &centre) {
    std::map<std::string,Equ> m = std::map<std::string,Equ>();
    
    m["tl"] = _rotate_getPerpendicular(_rotate_getLinearEquation(originals["tl"],projections["tl"]),centre["tl"]);
    m["tr"] = _rotate_getPerpendicular(_rotate_getLinearEquation(originals["tr"],projections["tr"]),centre["tr"]);
    m["bl"] = _rotate_getPerpendicular(_rotate_getLinearEquation(originals["bl"],projections["bl"]),centre["bl"]);
    m["br"] = _rotate_getPerpendicular(_rotate_getLinearEquation(originals["br"],projections["br"]),centre["br"]);
    
    return m;
}

cv::Point Utils::_rotate_getColisionPoint(Equ e1,Equ e2) {
    
    cv::Point2d p = cv::Point2d(0,0);
    
    if(e1.isHorizontal && e2.isVertical) {
        
        p.x = e2.c;
        p.y = e1.c;
        
    } else if(e1.isVertical && e2.isHorizontal) {
        
        p.x = e1.c;
        p.y = e2.c;
        
    } else if(e1.isHorizontal) {
        
        p.y = e1.c;
        p.x = (p.y-e2.c)/e2.m;
        
    } else if(e2.isHorizontal) {
        
        p.y = e2.c;
        p.x = (p.y-e1.c)/e1.m;
        
    } else if(e1.isVertical) {
        
        p.x = e1.c;
        p.y = p.x*e2.m+e2.c;
        
    } else if(e2.isVertical) {
        
        p.x = e2.c;
        p.y = p.x*e1.m+e1.c;
        
    } else {
        
        p.y = fabs(e1.c*e2.m-e2.c*e1.m)/fabs(e1.m-e2.m);
        p.x = (p.y-e1.c)/e1.m;
        
    }
    
    return cv::Point(round(p.x),round(p.y));
}

void Utils::rotate(cv::Mat &src, cv::Mat &dst, double degree) {
    cv::Mat copy,rot_mat;
    std::map<std::string,int> margins;
    cv::Size img,canvas,workspace;
    double h,angle;
    std::map<std::string,cv::Point> original, projection, centre;
    std::map<std::string,Equ> pEqu;
    cv::Point2f src_centre;
    
    img = Size(src.cols,src.rows);
    
    h = sqrt(pow(img.width/2.0,2) + pow(img.height/2.0,2));
    angle = atan((double)img.height/img.width);
    
    canvas = _rotate_newCanvasSize(degree,angle,h);
    margins = _rotate_extraMargins(img,canvas);
    cv::copyMakeBorder(src,copy,margins["top"],margins["bottom"],margins["left"],margins["right"],cv::BORDER_CONSTANT,cv::Scalar(0,0,0));
    workspace = cv::Size(copy.cols,copy.rows);
    
    original = _rotate_getCorners(img,margins);
    projection = _rotate_getProjectedCorners(canvas,h,degree,angle);
    centre = _rotate_getCentreBetweenOriginalsAndProjections(original,projection);
    pEqu = _rotate_getPerpendicularLinearEquation(original,projection,centre);
    
    if(img.width > canvas.width || img.height > canvas.height) {
        cv::Point p = _rotate_getColisionPoint(pEqu["tl"],pEqu["tr"]);
        src_centre = cv::Point2f(p.x,p.y);
    } else {
        src_centre = cv::Point2f(canvas.width/2.0,canvas.height/2.0);
    }
    
    dst.create(canvas.height,canvas.width,src.channels());

    rot_mat = cv::getRotationMatrix2D(src_centre, degree, 1.0);
    cv::warpAffine(copy, dst, rot_mat,canvas);
    
}
