#pragma once

#include <stdint.h>

#define NEWTON_PORT 4420
#define N_APRILTAG_COUNT 16
#define N_IMAGESTREAM_BPP 3
#define N_CAMERA_FOCAL_LENGTH 1200

struct NAprilTag
{
	int32_t id = 0;;
	float error;
	float orientation[3 * 3]; // row major, Z is normal direction
	float translation[3]; // x y z
};


enum NSubscribeFlags
{
	N_SUBSCRIBE_APRILTAGS   = 0x1,
	N_SUBSCRIBE_IMAGESTREAM = 0x2,
};


enum NMessageType
{
	N_MSG_NONE = 0,
	N_MSG_CONFIGURE,	 // For Newton (the Pi)
	N_MSG_APRILTAGS,	 // For Atlas (the Robot)
	N_MSG_IMAGESTREAM, 	 // For Atlas (the Robot)
};

struct NMessageHeader
{
	int32_t type;
	int32_t length;
};

// message types
struct NMessage_Configure
{
	uint16_t subscribe_flags;
};

struct NMessage_AprilTags
{
	int32_t num_tags;
	NAprilTag tags[N_APRILTAG_COUNT];
};

struct NMessage_ImageStreamHeader
{
	int32_t width;
	int32_t height;
};
