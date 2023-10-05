#include "Types.h"

float time_now() {
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    return time_us / 1000000.0;
}

float to_rads(float degrees) {
	return degrees * 3.1415926f/180.0f;
}

float wrap(float angle) {
	static constexpr float PI = 3.1415926f;
	float theta = std::fmod(angle, 2*PI);
	if(theta > PI) theta = theta - 2*PI;
	else if(theta < -PI) theta = theta + 2*PI;
	return theta;
}