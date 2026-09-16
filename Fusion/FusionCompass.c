/**
 * @file FusionCompass.c
 * @author Seb Madgwick
 * @brief Tilt-compensated compass to calculate magnetic heading.
 */

//------------------------------------------------------------------------------
// Includes

#include "FusionCompass.h"
#include <math.h>

//------------------------------------------------------------------------------
// Functions

/**
 * @brief Calculates magnetic heading.
 * @param accelerometer Accelerometer in any calibrated units.
 * @param magnetometer Magnetometer in any calibrated units.
 * @param convention Earth axes convention.
 * @return Magnetic heading in degrees.
 */
float FusionCompass(const FusionVector accelerometer, const FusionVector magnetometer, const FusionConvention convention) {
    // Double atan2: Meshtastic firmware already links it, atan2f would add a second (float) implementation.
    switch (convention) {
        case FusionConventionNwu: {
            const FusionVector west = FusionVectorNormalise(FusionVectorCross(accelerometer, magnetometer));
            const FusionVector north = FusionVectorNormalise(FusionVectorCross(west, accelerometer));
            return FusionRadiansToDegrees((float) atan2(west.axis.x, north.axis.x));
        }
        case FusionConventionEnu: {
            const FusionVector west = FusionVectorNormalise(FusionVectorCross(accelerometer, magnetometer));
            const FusionVector north = FusionVectorNormalise(FusionVectorCross(west, accelerometer));
            const FusionVector east = FusionVectorScale(west, -1.0f);
            return FusionRadiansToDegrees((float) atan2(north.axis.x, east.axis.x));
        }
        case FusionConventionNed: {
            const FusionVector up = FusionVectorScale(accelerometer, -1.0f);
            const FusionVector west = FusionVectorNormalise(FusionVectorCross(up, magnetometer));
            const FusionVector north = FusionVectorNormalise(FusionVectorCross(west, up));
            return FusionRadiansToDegrees((float) atan2(west.axis.x, north.axis.x));
        }
    }
    return 0.0f; // avoid compiler warning
}

//------------------------------------------------------------------------------
// End of file
