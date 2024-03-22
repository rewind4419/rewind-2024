#pragma once

#include "fennec/maths.h"

struct AprilTagAnchor
{
	v2 location;
	v2 normal;
};

// in metres!

AprilTagAnchor APRIL_TAG_ANCHORS [CFG_APRIL_TAG_COUNT] = {
	{ { 593.68 * INCH_TO_METER, 9.68 * INCH_TO_METER }, rotate( {1, 0}, degToRad(120)) },
	{ { 637.21 * INCH_TO_METER, 34.79 * INCH_TO_METER  }, rotate( {1, 0}, degToRad(120)) },
	{ { 652.73 * INCH_TO_METER, 196.17 * INCH_TO_METER }, rotate( {1, 0}, degToRad(180)) },
	{ { 652.73 * INCH_TO_METER, 218.42 * INCH_TO_METER }, rotate( {1, 0}, degToRad(180)) },
	{ { 578.77 * INCH_TO_METER, 323.00 * INCH_TO_METER }, rotate( {1, 0}, degToRad(270)) }, // four
	{ { 72.5 * INCH_TO_METER, 323.00 * INCH_TO_METER }, rotate( {1, 0}, degToRad(270)) },
	{ { -1.50 * INCH_TO_METER, 218.42 * INCH_TO_METER }, rotate( {1, 0}, degToRad(0)) },
	{ { -1.50 * INCH_TO_METER, 196.17 * INCH_TO_METER }, rotate( {1, 0}, degToRad(0)) }, // seven
	{ { 14.02 * INCH_TO_METER, 34.79 * INCH_TO_METER }, rotate( {1, 0}, degToRad(60)) },
	{ { 57.54 * INCH_TO_METER, 9.68 * INCH_TO_METER }, rotate( {1, 0}, degToRad(60)) },
	{ { 468.69 * INCH_TO_METER, 146.19 * INCH_TO_METER }, rotate( {1, 0}, degToRad(300)) },
	{ { 468.69 * INCH_TO_METER, 177.10 * INCH_TO_METER }, rotate( {1, 0}, degToRad(60)) },
	{ { 441.74 * INCH_TO_METER, 161.62 * INCH_TO_METER }, rotate( {1, 0}, degToRad(180)) },
	{ { 209.48 * INCH_TO_METER, 161.62 * INCH_TO_METER }, rotate( {1, 0}, degToRad(0)) },
	{ { 182.73 * INCH_TO_METER, 177.10 * INCH_TO_METER }, rotate( {1, 0}, degToRad(120)) },
	{ { 182.73 * INCH_TO_METER, 146.19 * INCH_TO_METER }, rotate( {1, 0}, degToRad(240)) },
};

