#pragma once

class LegacyPose {
    public:
        float x;
        float y;
        float theta;
        LegacyPose();
        LegacyPose(float x, float y, float theta = 0);
        LegacyPose operator+(const LegacyPose& other) const;
        LegacyPose operator-(const LegacyPose& other) const;
        float operator*(const LegacyPose& other) const;
        LegacyPose operator*(const float& other) const;
        LegacyPose operator/(const float& other) const;
        LegacyPose lerp(LegacyPose other, float t) const;
        float distance(LegacyPose other) const;
        float angle(LegacyPose other) const;
        LegacyPose rotate(float angle) const;
};

class Point {
    public:
        double x;
        double y;
        Point();
        Point(double x, double y);
        Point operator+(const Point& other) const;
        Point operator-(const Point& other) const;
        double operator*(const Point& other) const;
        Point operator*(const double& other) const;
        Point operator/(const double& other) const;
        double distance(Point other) const;
};