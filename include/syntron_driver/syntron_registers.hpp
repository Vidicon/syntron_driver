namespace Syntron
{
    namespace ParamReg
    {
        enum APP_MODE // Fn 000
        {
            NORMAL = 0,
            MODBUS = 1,
            CANBUS = 2,
        };
        
        enum NORMAL_CONTROL_MODE // Fn 001
        {
            SPEED_EXTERN = 0,
            SPEED_INTERNAL = 1,
            EXTERN_POSITION_PULSE = 2,
            JOG = 3,
            TORQUE = 4,
            SPEED_PULSE = 11,
        };

        enum BUS_CONTROL_MODE // Fn 003
        {
            IDLE = 0,
            TORQUE = 1,
            SPEED = 2,
            POSITION_REL_TIME = 3,
            POSITION_ABS_TIME = 4,
            POSITION_REL_SPEED = 5,
            POSITION_ABS_SPEED = 6,
        };

        enum FACTORY_RESET // Fn 007
        {
            RESET = 1,
        };

        enum REVERSE_SPEED_FEEDBACK // Fn 00B
        {
            NORMAL = 0,
            REVERCE = 1,
        };

        enum SERVO_ENABLE // Fn 010
        {
            DISABLE = 0,
            ENABLE = 1,
        };
        
        enum ALARM_CLEAR // Fn 011
        {
            CLEAR = 1,
        };

        enum EMERGENCY_STOP // Fn 012
        {
            RELEASE = 0,
            STOP = 1,
        };

    }
}