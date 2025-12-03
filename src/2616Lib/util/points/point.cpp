#include "main.h"

Point::Point():
  x(0), y(0) {}

Point::Point(double x, double y):
  x(x), y(y) {}

//Check if a Point equals another
bool Point::operator==(const Point &other) const {
  return this->x == other.x && this->y == other.y;
}

//Check if a Point is different from another
bool Point::operator!=(const Point &other)const{
  return !operator==(other);
}

//Add two Points together
Point Point::operator+(const Point &other) const{
  return Point(this->x + other.x, this->y + other.y);
}

//Subtract two Points from each other
Point Point::operator-(const Point &other) const{
  return Point(this->x - other.x, this->y - other.y);
}

//Multiply a Point by a constant
Point Point::operator*(double other) const{
  return Point(this->x * other, this->y * other);
}

//Divide a Point by a constant
Point Point::operator/(double other) const{
  return Point(this->x / other, this->y / other);
}

double roundToNDigits(double value, int n) {
    double scale = std::pow(10.0, n);
    return std::round(value * scale) / scale;
}

// Overload round function for Point
Point round(const Point& p, int n) {
    return Point(roundToNDigits(p.x, n), roundToNDigits(p.y, n));
}
