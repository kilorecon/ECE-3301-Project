#ifndef Quaternion_Math
#define Quaternion_Math
#include "MPU6050_6Axis_MotionApps20.h"

// Quaternion/Quaternion multiplication
Quaternion operator*(const Quaternion& q1, const Quaternion& q2) {  
    Quaternion result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

// Quaternion/Scalar multiplication
Quaternion operator*(const Quaternion& q1, float scalar) {
    return {q1.w * scalar, q1.x * scalar, q1.y * scalar, q1.z * scalar};
}

// Quaternion addition
Quaternion operator+(const Quaternion& q1, const Quaternion& q2) {
    Quaternion result;
    result.w = q1.w + q2.w;
    result.x = q1.x + q2.x;
    result.y = q1.y + q2.y;
    result.z = q1.z + q2.z;
    return result;
}

/*// Quaternion subtraction
Quaternion operator-(Quaternion& q1, Quaternion& q2) {
    return {q1.w - q2.w, q1.x - q2.x, q1.y - q2.y, q1.z - q2.z};
}*/

// Quaternion negative
Quaternion operator-(Quaternion& q) {
    return {-q.w, -q.x, -q.y, -q.z};
}

// Quaternion conjugate
Quaternion getConjugate(Quaternion &q) {
    return {q.w, -q.x, -q.y, -q.z};
}

Quaternion normalize(Quaternion q) {
  float norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (norm > 0.0) {
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
  }
  return {q.w, q.x, q.y, q.z};
}

#endif