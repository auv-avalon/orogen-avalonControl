#ifndef AVALON_CONTROL_HPP
#define AVALON_CONTROL_HPP

#include <pidcontroller.h>
#include <base/time.h>

namespace avalon_control
{
    enum MOTCON_CHANNELS
    {
        MIDDLE_HORIZONTAL = 5,
        MIDDLE_VERTICAL   = 1,
        REAR_HORIZONTAL   = 0,
        REAR_VERTICAL     = 4,
        RIGHT = 2,
        LEFT  = 3 
    };

    enum MOTCON_CHANNEL_DIRECTIONS
    {
        DIR_MIDDLE_HORIZONTAL = 1,
        DIR_MIDDLE_VERTICAL   = -1,
        DIR_REAR_HORIZONTAL   = -1,
        DIR_REAR_VERTICAL     = 1,
        DIR_RIGHT = 1,
        DIR_LEFT  = -1
    };

    struct MotionControllerState
    {
        PIDControllerState z_pid;
        PIDControllerState heading_pid;
        PIDControllerState pitch_pid;
    };

    struct SpeedControllerState
    {
        PIDControllerState z_pid;
        PIDControllerState heading_pid;
        PIDControllerState pitch_pid;
    };

    struct SpeedCommand
    {
        base::Time time;
        double linear_speeds[3]; //! x, y and z speeds in m/s
        double rotation_speed;   //! rotation around the yaw axis in radians/s

#ifndef __orogen
        SpeedCommand()
            : rotation_speed(0)
        {
            linear_speeds[0] = linear_speeds[1] = linear_speeds[2] = 0;
        }
#endif
    };
}

#endif

