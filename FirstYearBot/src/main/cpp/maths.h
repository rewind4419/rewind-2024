#pragma once

// Note:
// *** DONT RENAME THIS FILE TO MATH.H!! ***
// math.h is taken so renaming by the C standard library,
// which will make this one will cause conflicts



// @Todo inline all of these


#define PI 3.14159265358979
#define INCH_TO_METER (1.0 / 39.37)

float sign(float x);

float degToRad(float degrees);
float radToDeg(float radians);


#define MIN(a, b) 	((a<b)?(a):(b))
#define MAX(a, b) 	((a>b)?(a):(b))
#define CLAMP(x, a, b) (MAX(MIN(x, b), a))




struct v2
{
	float x;
  float y;    
};



// @Todo inline all of these

// Preforms a clockwise rotation in radians on the 2d vector
v2 rotate(v2 vector, float angle);

v2 leftPerpendicular(v2 vector);
v2 rightPerpendicular(v2 vector);

inline float mix(float a, float b, float t) { return a * (1 - t) + b * t; }
inline v2 mix(v2 a, v2 b, float t) 
{
	return v2 {
		mix(a.x, b.x, t),
		mix(a.y, b.y, t),
	};
}

float dot(v2 a, v2 b);
float length(v2 v);
float lengthSq(v2 v);

inline v2 operator-(v2 a) { return v2 { -a.x, -a.y }; }

inline v2 operator+(v2 a, v2 b) { return v2 { a.x + b.x, a.y + b.y }; }
inline v2 operator-(v2 a, v2 b) { return v2 { a.x - b.x, a.y - b.y }; }
inline v2 operator*(v2 a, v2 b) { return v2 { a.x * b.x, a.y * b.y }; }
inline v2 operator/(v2 a, v2 b) { return v2 { a.x / b.x, a.y / b.y }; }

inline v2 operator*(v2 a, float b) { return v2 { a.x * b, a.y * b }; }
inline v2 operator/(v2 a, float b) { return v2 { a.x / b, a.y / b }; }


inline v2 normalize(v2 v) { float l = length(v); if (l == 0) l = 1; return v / v2 { l, l }; }






struct v3
{
  float x;
  float y;
	float z;    
};




struct Pose
{
  v2    position;
  float rotation;
};



bool solveIK(v2 solutions[], float arm1_length, float arm2_length, v2 target_point);