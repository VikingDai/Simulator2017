#pragma once

#include <math.h>
#include <vector>
using namespace std;

class Point2D
{

 public:
  double x;
  double y;

  Point2D();
  Point2D(double x_in, double y_in);
  Point2D operator*(const double& other);
  Point2D operator+(const Point2D& other);
  Point2D operator-(const Point2D& other);
  Point2D rotate(double angle);
};

class Vector2D
{

 public:
  double x;
  double y;

  Vector2D();
  Vector2D(double x_in, double y_in);

  Point2D operator+(const Point2D& other);
  Point2D operator-(const Point2D& other);

  Vector2D operator+(const Vector2D& other);
  Vector2D operator-(const Vector2D& other);
  Vector2D operator*(const double& other);

  double dot(Vector2D a);
  double cross(Vector2D a);
  double norm();
  void scale(double s);
  void normalize();
};

class LineSegment
{

 public:
  Point2D a, b;

  LineSegment();
  LineSegment(Point2D a_in, Point2D b_in)
    {
      a = a_in;
      b = b_in;
    }
		

  int getIntersection(LineSegment seg_in, Point2D& intersectPoint);
};

Vector2D toVector2D(Point2D a, Point2D b);

Point2D getClosestIntersect(Point2D a, Point2D b, Point2D p);

int getLineCircleIntersections(Point2D a, Point2D b, Point2D p, double radius, vector<Point2D> &vectorResult);

int isOkAddIntermediateWayPoint(Point2D p, Point2D a, Point2D b);

int sideOfLine(Point2D p1, Point2D p2, Point2D evaluatePoint);
