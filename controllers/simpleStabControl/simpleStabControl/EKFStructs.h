#ifndef EKFStructs_h
#define EKFStructs_h


/**
 * AccelMeasurement - A non-generic class for holding accelerometer values
 *                    required (by motion) for filtering accel sensor values.
 */

struct ZmpTimeUpdate{
	float cur_zmp_x;
	float cur_zmp_y;
};

struct ZmpMeasurement {
    float comX;
    float comY;
    float accX;
    float accY;
};


/**
 * AccelMeasurement - A non-generic class for holding accelerometer values
 *                    required (by motion) for filtering accel sensor values.
 */
struct AccelMeasurement {
    float x;
    float y;
    float z;
};

/**
 * AngleMeasurement - a class for holding angleX, angleY
 */

struct AngleMeasurement{
  float angleX;
  float angleY;
};




#endif