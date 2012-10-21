/**
 * View steps used to get a rotation without cropping in OpenCV
 * 
 * @version 08/05/2012
 * @author Samuel Molinari
 */

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

double round(double number) {
    return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
}

double degreeToRadian(double degree) {
    return (M_PI*degree)/180.0;
}

double radianToDegree(double radian) {
    return (180*radian)/M_PI;
}

Size rotationNewCanvasSize(double degree,double angle,double h) {
    Size canvas(0,0);
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

double solveEquationY(Equ e,double x) {
    
    cout << x << endl;
    
    if(e.isVertical) return x;
    
    return e.m*x+e.c;
}

double solveEquationX(Equ e, double y) {
    
    if(e.isVertical) return e.c;
    if(e.isHorizontal) return 0;

    return (y-e.c)/e.m;
}

map<string,int> rotationExtraMargins(Size &original, Size &newSize) {
    
    map<string,int> m = map<string,int>();
    
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

map<string,Point> getCorners(Size &original,map<string,int> &margins) {
    map<string,Point> m = map<string,Point>();
    
    m["tl"] = Point(margins["left"],margins["top"]);
    m["tr"] = Point(margins["left"]+original.width,margins["top"]);
    m["bl"] = Point(margins["left"],margins["top"]+original.height);
    m["br"] = Point(margins["left"]+original.width,margins["top"]+original.height);
    
    return m;
    
}

map<string,Point> getProjectedCorners(Size &s,double h,double degree,double angle) {
    
    map<string,Point> m = map<string,Point>();
    double radian = degreeToRadian(degree);
    int top,left;
    
    m["tl"] = Point(
                round(s.width/2.0+h*cos(M_PI-angle+radian)),
                round(s.height/2.0-h*sin(M_PI-angle+radian)));
    
    top = m["tl"].y;
    left = m["tl"].x;
    
    m["tr"] = Point(
                round(s.width/2.0+h*cos(angle+radian)),
                round(s.height/2.0-h*sin(angle+radian)));
    
    if(top>m["tr"].y) top=m["tr"].y;
    if(left>m["tr"].x) left=m["tr"].x;
    
    m["br"] = Point(
                round(s.width/2.0+h*cos(-angle+radian)),
                round(s.height/2.0-h*sin(-angle+radian)));
    
    if(top>m["br"].y) top=m["br"].y;
    if(left>m["br"].x) left=m["br"].x;
    
    m["bl"] = Point(
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

Point getCentreBetweenPoints(Point &a, Point &b) {
    Point c(0,0);
    double tmp;
    
    tmp = abs(a.x-b.x)/2.0;
    
    if(b.x <= a.x) c.x = round(tmp+b.x);
    else c.x = round(tmp+a.x);
    
    tmp = abs(a.y-b.y)/2.0;
    
    if(b.y <= a.y) c.y = round(tmp+b.y);
    else c.y = round(tmp+a.y);
    
    return c;
}

map<string,Point> getCentreBetweenOriginalsAndProjections(map<string,Point> &originals,map<string,Point> &projections) {
    map<string,Point> m = map<string,Point>();
    
    m["tl"] = getCentreBetweenPoints(originals["tl"],projections["tl"]);
    m["tr"] = getCentreBetweenPoints(originals["tr"],projections["tr"]);
    m["bl"] = getCentreBetweenPoints(originals["bl"],projections["bl"]);
    m["br"] = getCentreBetweenPoints(originals["br"],projections["br"]);
    
    return m;
    
}

Equ getLinearEquation(Point &a,Point &b) {
    
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

Equ getPerpendicular(Equ e,Point p) {
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

map<string,Equ> getLinearEquationBetweenOriginalsAndProjections(map<string,Point> &originals,map<string,Point> &projections) {
    map<string,Equ> m = map<string,Equ>();
    
    m["tl"] = getLinearEquation(originals["tl"],projections["tl"]);
    m["tr"] = getLinearEquation(originals["tr"],projections["tr"]);
    m["bl"] = getLinearEquation(originals["bl"],projections["bl"]);
    m["br"] = getLinearEquation(originals["br"],projections["br"]);
    
    return m;
}

map<string,Equ> getPerpendicularLinearEquation(map<string,Point> &originals,map<string,Point> &projections,map<string,Point> &centre) {
    map<string,Equ> m = map<string,Equ>();
    
    m["tl"] = getPerpendicular(getLinearEquation(originals["tl"],projections["tl"]),centre["tl"]);
    m["tr"] = getPerpendicular(getLinearEquation(originals["tr"],projections["tr"]),centre["tr"]);
    m["bl"] = getPerpendicular(getLinearEquation(originals["bl"],projections["bl"]),centre["bl"]);
    m["br"] = getPerpendicular(getLinearEquation(originals["br"],projections["br"]),centre["br"]);
    
    return m;
}

Point getColisionPoint(Equ e1,Equ e2) {
    
    Point2d p = Point2d(0,0);
    
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
    
    return Point(round(p.x),round(p.y));
}

int main(int argc, char *argv[])  
{
    
    Mat image = Mat(imread(argv[1]));
    Mat copy,schema,rot_mat,dst;
    map<string,int> margins;
    Size img,canvas,workspace;
    double h,angle,degree;
    map<string,Point> original, projection, centre;
    map<string,Equ> pEqu;
    Point2f src_centre;
    int speed = 30;
    int type = 0;
    
    if(argc > 2) {
        speed = atoi(argv[2]);
        cout << speed << endl;
    }
    
    if(argc > 3) {
        type = atoi(argv[3]);
    }
    
    img = Size(image.cols,image.rows);
    h = sqrt(pow(img.width/2.0,2) + pow(img.height/2.0,2));
    angle = atan((double)img.height/img.width);
    
    namedWindow("Rotate");
    imshow("Rotate",image);
    
    if(type == 0) {
        namedWindow("Schema"); 
        imshow("Schema",image);
    }
    
    waitKey(0);
    
    for(degree=0;degree<360;degree++) {
    
        canvas = rotationNewCanvasSize(degree,angle,h);
        margins = rotationExtraMargins(img,canvas);
        copyMakeBorder(image,copy,margins["top"],margins["bottom"],margins["left"],margins["right"],BORDER_CONSTANT,Scalar(0,0,0));
        copy.copyTo(schema);
        workspace = Size(copy.cols,copy.rows);
        
        original = getCorners(img,margins);
        projection = getProjectedCorners(canvas,h,degree,angle);
        centre = getCentreBetweenOriginalsAndProjections(original,projection);
        pEqu = getPerpendicularLinearEquation(original,projection,centre);
        
        cv::ellipse(schema,original["tl"],Size(1,1),0,0,360,Scalar(0,0,255),3);
        cv::ellipse(schema,original["tr"],Size(1,1),0,0,360,Scalar(0,0,255),3);
        cv::ellipse(schema,original["bl"],Size(1,1),0,0,360,Scalar(0,0,255),3);
        cv::ellipse(schema,original["br"],Size(1,1),0,0,360,Scalar(0,0,255),3);
        cv::line(schema,original["tl"],original["tr"],Scalar(0,0,255),1);
        cv::line(schema,original["tr"],original["br"],Scalar(0,0,255),1);
        cv::line(schema,original["br"],original["bl"],Scalar(0,0,255),1);
        cv::line(schema,original["bl"],original["tl"],Scalar(0,0,255),1); 
        
        cv::ellipse(schema,projection["tl"],Size(1,1),0,0,360,Scalar(0,255,255),3);
        cv::ellipse(schema,projection["tr"],Size(1,1),0,0,360,Scalar(0,255,255),3);
        cv::ellipse(schema,projection["bl"],Size(1,1),0,0,360,Scalar(0,255,255),3);
        cv::ellipse(schema,projection["br"],Size(1,1),0,0,360,Scalar(0,255,255),3);
        cv::line(schema,projection["tl"],projection["tr"],Scalar(0,255,255),1);
        cv::line(schema,projection["tr"],projection["br"],Scalar(0,255,255),1);
        cv::line(schema,projection["br"],projection["bl"],Scalar(0,255,255),1);
        cv::line(schema,projection["bl"],projection["tl"],Scalar(0,255,255),1);
        
        
        cv::line(schema,original["tl"],projection["tl"],Scalar(0,192,255),1);
        cv::line(schema,original["tr"],projection["tr"],Scalar(0,192,255),1);
        cv::line(schema,original["br"],projection["br"],Scalar(0,192,255),1);
        cv::line(schema,original["bl"],projection["bl"],Scalar(0,192,255),1);
        
        cv::ellipse(schema,centre["tl"],Size(1,1),0,0,360,Scalar(255,162,0),3);
        cv::ellipse(schema,centre["tr"],Size(1,1),0,0,360,Scalar(255,162,0),3);
        cv::ellipse(schema,centre["bl"],Size(1,1),0,0,360,Scalar(255,162,0),3);
        cv::ellipse(schema,centre["br"],Size(1,1),0,0,360,Scalar(255,162,0),3);
        
        cv::line(schema,centre["tl"],getColisionPoint(pEqu["tl"],pEqu["tr"]),Scalar(255,162,0),1);
        cv::line(schema,centre["tr"],getColisionPoint(pEqu["tr"],pEqu["br"]),Scalar(255,162,0),1);
        cv::line(schema,centre["br"],getColisionPoint(pEqu["br"],pEqu["bl"]),Scalar(255,162,0),1);
        cv::line(schema,centre["bl"],getColisionPoint(pEqu["bl"],pEqu["tl"]),Scalar(255,162,0),1);
        
        cv::ellipse(schema,getColisionPoint(pEqu["tl"],pEqu["tr"]),Size(1,1),0,0,360,Scalar(102,0,255),3);
        cv::ellipse(schema,getColisionPoint(pEqu["tr"],pEqu["br"]),Size(1,1),0,0,360,Scalar(102,0,255),3);
        cv::ellipse(schema,getColisionPoint(pEqu["br"],pEqu["bl"]),Size(1,1),0,0,360,Scalar(102,0,255),3);
        cv::ellipse(schema,getColisionPoint(pEqu["bl"],pEqu["tl"]),Size(1,1),0,0,360,Scalar(102,0,255),3);

        if(img.width > canvas.width || img.height > canvas.height) {
            Point p = getColisionPoint(pEqu["tl"],pEqu["tr"]);
            src_centre = Point2f(p.x,p.y);
        } else {
            src_centre = Point2f(canvas.width/2.0,canvas.height/2.0);
        }
        
        cv::ellipse(schema,Point(round(src_centre.x),round(src_centre.y)),Size(1,1),0,0,360,Scalar(102,0,255),3);
        
        
        if(type == 0) {
            imshow("Schema",schema);
            dst.create(canvas.height,canvas.width,image.channels());
            rot_mat = getRotationMatrix2D(src_centre, degree, 1.0);
            cv::warpAffine(copy, dst, rot_mat,canvas);
            imshow("Rotate",dst);
        } else if(type == 1) {
            
            dst.create(image.rows,image.cols,image.channels());
            rot_mat = getRotationMatrix2D(Point2f(image.cols/2.0,image.rows/2.0), degree, 1.0);
            cv::warpAffine(image, dst, rot_mat,canvas);
            imshow("Rotate",dst);
            
        } else if(type == 2) {
            
            dst.create(image.rows,image.cols,image.channels());
            rot_mat = getRotationMatrix2D(Point2f(image.cols/2.0,image.rows/2.0), degree, 1.0);
            cv::warpAffine(image, dst, rot_mat,Size(image.cols,image.rows));
            imshow("Rotate",dst);
            
        }
        
        
        
        
        
        waitKey(speed);
        
    }
    
    return 0;
}
