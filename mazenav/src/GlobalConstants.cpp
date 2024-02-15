#include "GlobalConstants.h"
#include <cmath>

const int GRID_SIZE {300};

// Radius of the wheel
#warning untuned constant
const int WHEEL_RADIUS {40};

// Horizontal spacing between the wheels.
#warning untuned constant
const int WHEEL_SPACING {170};

// Vertical spacing between the wheels
#warning untuned constant
const int WHEELBASE {190};

// Distance from middle of robot to the wheel contact point
const int WHEEL_TURN_RADIUS {static_cast<int>(sqrt(WHEEL_SPACING*WHEEL_SPACING/4 + WHEELBASE*WHEELBASE/4))};

// How much the wheel will contribute to the rotation. Is one if the wheelbase is very large in relation to the wheel spacing.
const double WHEEL_TURN_CONTRIBUTION_FACTOR {cos(atan(static_cast<double>(WHEELBASE)/WHEEL_SPACING))};