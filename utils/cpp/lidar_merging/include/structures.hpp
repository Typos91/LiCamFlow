#ifndef STRUCTURES_HPP
#define STRUCTURES_HPP

#include <cmath>
#include <iostream>
#include <fstream> 
#include <sstream>
#include <Eigen/Dense>

struct Quaternion {
    double w, x, y, z; // w + xi + yj + zk

    // Constructors
    Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    // Equality operator
    bool operator==(const Quaternion& q) const {
        return (w == q.w) && (x == q.x) && (y == q.y) && (z == q.z);
    }

    // Addition
    Quaternion operator+(const Quaternion& q) const {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }

    // Multiplication (Hamilton product)
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    // Scalar multiplication
    Quaternion operator*(double s) const {
        return Quaternion(w * s, x * s, y * s, z * s);
    }

    // Streaming data 
    friend std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
        os << "(" << q.w << ", " << q.x << "i, " << q.y << "j, " << q.z << "k)";
        return os;
    }

    // Conjugate
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    // Norm
    double norm() const {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    // Normalization
    Quaternion normalize() const {
        double n = norm();
        return Quaternion(w / n, x / n, y / n, z / n);
    }

    // Inverse
    Quaternion inverse() const {
        double n = norm();
        return conjugate() * (1.0 / (n * n));
    }

    // Rotation of a vector (3D) by this quaternion
    Quaternion rotate(const Quaternion& v) const {
        Quaternion p = (*this) * v * conjugate();
        return p;
    }

    // Print
    void print() const {
        std::cout << "(" << w << ", " << x << "i, " << y << "j, " << z << "k)" << std::endl;
    }

    // Static method to create a quaternion from axis-angle
    static Quaternion fromAxisAngle(double angle, double ax, double ay, double az) {
        double halfAngle = angle / 2.0;
        double sinHalf = std::sin(halfAngle);
        return Quaternion(
            std::cos(halfAngle),
            ax * sinHalf,
            ay * sinHalf,
            az * sinHalf
        ).normalize();
    }

    // Convert the quaternion to a 3x3 Eigen rotation matrix
    Eigen::Matrix3d toRotationMatrix() const {
        Eigen::Matrix3d R;
        double xx = x * x, yy = y * y, zz = z * z;
        double xy = x * y, xz = x * z, yz = y * z;
        double wx = w * x, wy = w * y, wz = w * z;

        R(0, 0) = 1.0 - 2.0 * (yy + zz);
        R(0, 1) = 2.0 * (xy - wz);
        R(0, 2) = 2.0 * (xz + wy);

        R(1, 0) = 2.0 * (xy + wz);
        R(1, 1) = 1.0 - 2.0 * (xx + zz);
        R(1, 2) = 2.0 * (yz - wx);

        R(2, 0) = 2.0 * (xz - wy);
        R(2, 1) = 2.0 * (yz + wx);
        R(2, 2) = 1.0 - 2.0 * (xx + yy);

        return R;
    }
};

#endif // STRUCTURES_HPP