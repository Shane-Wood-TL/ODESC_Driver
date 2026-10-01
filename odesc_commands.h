/**
 * @files Contains the command id for various odesc commands
 * @author Shane Wood
 */
#ifndef __odesc_commands__
#define __odesc_commands__

#define Heartbeat 0x01 ///< odesc command id for get version
#define Estop 0x02 ///< odesc command id for estop
#define Get_Motor_Error 0x03  ///< odesc command id for get motor error
#define Get_Encoder_Error 0x04 ///< odesc command id for get encoder error
#define Get_Sensorless_Error 0x05 ///< odesc command id for get sensorless error
#define Set_Axis_Node_ID 0x06  ///< odesc command id for setting node id
#define Set_Axis_Requested_State 0x07 ///< odesc command id for setting axis state
#define Set_Axis_Startup_Config 0x08 ///< NOT IMPLEMENTED
#define Get_Encoder_Estimates 0x09 ///< odesc command id for getting encoder estimates
#define Get_Encoder_Count 0x0A ///< odesc command id for getting encoder counts
#define Set_Controller_Modes 0x0B ///< odesc command id for setting the controller mode
#define Set_Input_Pos 0x0C ///< odesc command id for setting input position
#define Set_Input_Vel 0x0D ///< odesc command id for setting input velocity
#define Set_Input_Torque 0x0E ///< odesc command id for setting input torque
#define Set_Limits 0x0F ///< odesc command id for setting velocity and current limits
#define Start_Anitcogging 0x10 ///< odesc command id for starting anti cogging
#define Set_Traj_Vel_Limit 0x11  ///< odesc command id for setting trajectory velocity limits
#define Set_Traj_Accel_Limits 0x12 ///< odesc command id for setting trajectory acceleration limits
#define Set_Traj_Inertia 0x13 ///< odesc command id for settings trajactory interia limits
#define Get_Iq 0x14  ///< odesc command id for getting the magnitude of current on each phase (IQ) both setpoint and measure
#define Get_Sensorless_Estimates 0x15 ///< odesc command id for getting sensorless estimates
#define Reboot 0x16  ///< odesc command id for rebooting + saving configuration
#define Get_Bus_Voltage_Current 0x17  ///< odesc command id for getting the voltage and current of the odesc
#define Clear_Errors 0x18 ///< odesc command id for clearing errors
#define Set_Linear_Count 0x19 ///< odesc command id for setting linear count
#define Set_Position_Gain 0x1A ///< odesc command id for setting position gain
#define Set_Vel_Gains 0x1B ///< odesc command id for setting velocity gains
#define Get_ADC_Voltage 0x1C ///< odesc command id for getting adc voltage
#define Get_Controller_Error 0x1D ///< odesc command id for

#endif