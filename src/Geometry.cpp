#include "Geometry.h"


Point2D::Point2D()
{
	x = 0;
	y = 0;
}

Point2D::Point2D(double x_in, double y_in)
{
	x = x_in;
	y = y_in;
}

Point2D Point2D::rotate(double angle)
{
	return Point2D(cos(angle)*x - sin(angle)*y, sin(angle)*x + cos(angle)*y);
}

Point2D Point2D::operator*(const double& other)
{
	return Point2D(x *other, y*other);
}

Point2D Point2D::operator+(const Point2D& other)
{
	return Point2D(x +other.x, y+other.y);
}

Point2D Point2D::operator-(const Point2D& other)
{
	return Point2D(x -other.x, y-other.y);
}

Vector2D::Vector2D()
{
	x = 0;
	y = 0;
}

Vector2D::Vector2D(double x_in, double y_in)
{
	x = x_in;
	y = y_in;
}

Vector2D Vector2D::operator+(const Vector2D&  other)
{
	return Vector2D(x + other.x, y + other.y);
}

Vector2D Vector2D::operator-(const Vector2D&  other)
{
	return Vector2D(x - other.x, y - other.y);
}

Vector2D Vector2D::operator*(const double& other)
{
	return Vector2D(x *other, y*other);
}

Point2D Vector2D::operator+(const Point2D& other)
{
	return Point2D(x + other.x, y + other.y);
}

Point2D Vector2D::operator-(const Point2D& other)
{
	return Point2D(x - other.x, y - other.y);
}

double Vector2D::dot(Vector2D a)
{
	return x*a.x + y*a.y;
}

double Vector2D::cross(Vector2D a)
{
	return x*a.y - y*a.x;
}

double Vector2D::norm()
{
	return sqrt(x*x + y*y);
}

void Vector2D::normalize()
{
	double normVal = norm();
	x = x/normVal;
	y = y/normVal;
}

void Vector2D::scale(double s)
{
	x = x*s;
	y = y*s;
}


Vector2D toVector2D(Point2D a, Point2D b)
{
	return Vector2D(b.x - a.x, b.y - a.y);
}

int LineSegment::getIntersection(LineSegment seg_in, Point2D& intersectPoint)
{
	
	Vector2D r = toVector2D(a, b);
	Vector2D s = toVector2D(seg_in.a, seg_in.b);

	double rCs = r.cross(s);
	Vector2D qp = toVector2D(a, seg_in.a);

	if(rCs != 0.0)
	{
		double u = qp.cross(r) / rCs;
		double t = qp.cross(s) / rCs;

		if(u <= 1 && u >= 0 && t <= 1 && t >= 0)
		{
			intersectPoint = r*u + a;
			return 1;
		}
		else
			return 0;
	}
	else
		return 0;

	
}

Point2D getClosestIntersectLine(Point2D a, Point2D b, Point2D p)
{
	//calculate vector from a to b and p
	Vector2D ab = toVector2D(a, b);
	Vector2D ap = toVector2D(a, p);

	double norm_ab = ab.norm();

	//project ap on ab
	double u = ab.dot(ap)/norm_ab;

	//Scale ab to reach perpendicualar intersect point
	Point2D c = ab*(u/norm_ab) + a;

	return c;

}

Point2D getClosestIntersectSegment(Point2D a, Point2D b, Point2D p)
{
	//calculate vector from a to b and p
	Vector2D ab = toVector2D(a, b);
	Vector2D ap = toVector2D(a, p);

	double norm_ab = ab.norm();

	//project ap on ab
	double u = ab.dot(ap)/norm_ab;

	if(u < 0.0)
		return a;
	else if(u > 0.0)
		return b;
	else
	{
		//Scale ab to reach perpendicualar intersect point
		Point2D c = ab*(u/norm_ab) + a;

		return c;
	}

}

int getLineCircleIntersections(Point2D a, Point2D b, Point2D p, double radius, vector<Point2D> &vectorResult)
{

	//Calulate intersection of the line from a to b and the circle with radius radius and center in p

	//calculate vector from a to b and p
	Vector2D ab = toVector2D(a, b);
	Vector2D ap = toVector2D(a, p);

	double norm_ab = ab.norm();

	//project ap on ab
	double u = ab.dot(ap)/norm_ab;
	

	//Scale ab to reach perpendicualar intersect point
	Point2D c = ab*(u/norm_ab) + a;

	double distance_p2c = toVector2D(p, c).norm();

	vectorResult.clear();

	if(radius*radius > distance_p2c*distance_p2c)
	{
		double distance_a2radiusintersect = sqrt(radius*radius - distance_p2c*distance_p2c);

		Point2D intersect1 = ab*(distance_a2radiusintersect/norm_ab) + c;
		Point2D intersect2 = ab*(-distance_a2radiusintersect/norm_ab) + c;

		vectorResult.push_back(intersect1);
		vectorResult.push_back(intersect2);
		return 1;
	}
	else
	{
		vectorResult.push_back(b);
		vectorResult.push_back(b);
		return 0;
	}
	
}

int isOkAddIntermediateWayPoint(Point2D p, Point2D a, Point2D b)
{
		const double distanceBetweenIntermediateNodes = 5;
		
		Vector2D ab = toVector2D(a, b);

		//check if point p is on the line segment between a and b. Assumed p is on the line
		double alpha = (p.y - a.y)/(b.y - a.y);
			
		double distanceSinceLastAdded = ab.norm()*alpha;

		if(alpha > 1.0 || alpha < 0.0)
			return 0;

		if(distanceSinceLastAdded >= distanceBetweenIntermediateNodes)
			return 1;
		else
			return 0;

}

int sideOfLine(Point2D p1, Point2D p2, Point2D evaluatePoint)
{
		//return 1 if point is on left side of vector pointing from p1 to p2
		if( (p2.x-p1.x)*(evaluatePoint.y-p1.y) - (p2.y-p1.y)*(evaluatePoint.x-p1.x) <= 0)
			return 1;
		else
			return 0;
}
