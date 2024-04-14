#include "GlobalConstants.h"
#include <cmath>

const int GRID_SIZE {300};
const int WALL_THICKNESS {15};

// Radius of the wheel
const int WHEEL_RADIUS {39};

// Horizontal spacing between the wheels.
const int WHEEL_SPACING {163};

// Vertical spacing between the wheels
const int WHEELBASE {95};

// Distance from middle of robot to the wheel contact point
const int WHEEL_TURN_RADIUS {static_cast<int>(sqrt(WHEEL_SPACING*WHEEL_SPACING/4 + WHEELBASE*WHEELBASE/4))};

// How much the wheel will contribute to the rotation. Is one if the wheelbase is very large in relation to the wheel spacing.
const double WHEEL_TURN_CONTRIBUTION_FACTOR {1.5*cos(atan(static_cast<double>(WHEELBASE)/WHEEL_SPACING))};


// ToF sensor offsets
// Robot centre to front sensors, Y direction
const double TOF_FY_OFFSET {82};
// Robot centre to front sensor, X direction
const double TOF_FX_OFFSET {50};
// Robot centre to back sensor, Y direction (positive)
const double TOF_BY_OFFSET {TOF_FY_OFFSET};
// Robot centre to side sensors, X direction
const double TOF_SX_OFFSET {68};
// Robot centre to side sensors, Y direction
const double TOF_SY_OFFSET {63};


// Lidar offsets
// Robot centre to lidar centre, X direction
const double LIDAR_X_OFFSET {0};
// Robot centre to lidar centre, Y direction
const double LIDAR_Y_OFFSET {9}; // Actually 9.2mm