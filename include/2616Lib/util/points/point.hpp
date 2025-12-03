#pragma once

class Point {
  public:
    Point();   
    Point(double x, double y);

    double x, y;

    bool operator==(const Point &other) const;
    bool operator!=(const Point &other) const;
    Point operator+(const Point &other) const;
    Point operator-(const Point &other) const;
    Point operator/(double other) const;
    Point operator*(double other) const;

};

//Allows multiplication in the form of `number * Point` where number can be of any type, since operator* only handles the form of `Point * double`
template <typename T> Point operator*(const T scale, const Point& p)
{
  return Point(p.x * scale, p.y * scale);
}


Point round(const Point& p, int n);