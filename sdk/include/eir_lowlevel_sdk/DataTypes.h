/**
 * @file DataTypes.h
 *
 * @brief This file contains the declarations of classes and structures related to robotics.
 *
 *  Copyright (c) 2025, EIR Co.Ltd. All rights reserved.
 */
#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <memory>
#include <string>
#include <vector>
/**
 * @struct ImuData
 *
 * @brief Structure representing IMU (Inertial Measurement Unit) data of a robot based on sensor feedback.
 *
 * This structure encapsulates IMU data including accelerometer, gyroscope, and quaternion.
 */
struct ImuState {
  double quat[4];       // IMU Quaternion
  double quat_est[4];   // IMU Estimated Quaternion
  double euler[3];      // IMU Euler Angles
  double acc[3];        // IMU Accelerometer
  double acc_est[3];    // IMU Estimated acc
  double gyro[3];       // IMU Gyroscope
  double mag[3];        // IMU Magnetism
};

/**
 * @struct ForceSensorState
 *
 * @brief Structure representing the state of a force sensor.
 *
 * This structure holds the current force data and the current force sensor state.
 *
 */
struct ForceSensorState {
  std::vector<double> fx;  // Force X
  std::vector<double> fy;  // Force Y  
  std::vector<double> fz;  // Force Z
  std::vector<double> mx;  // Torque X
  std::vector<double> my;  // Torque Y
  std::vector<double> mz;  // Torque Z
  std::vector<int16_t> code;    // The current code
};

/**
 * @struct TorsoForceState
 *
 * @brief Structure representing the state of the torso force sensor.
 *
 * This structure holds the current force data and the current torso force sensor state.
 *
 */
struct TorsoForceState {
  std::vector<double> cfrc;         // The current force data;
  std::vector<int16_t> cfrc_state;  // The current force sensor state;
};

/**
 * @struct EncoderState
 *
 * @brief Structure representing the state of the encoders.
 *
 * This structure holds the current encoder position and velocity.
 *
 */
struct EncoderState {
  std::vector<double> q;           // The current encoder position
  std::vector<double> dq;          // The current encoder velocity
  std::vector<int16_t> status;     // The current encoder status, 0: normal read, 1: zero set completed, 2: zero set failed
};

/**
 * @struct EncoderCmd
 *
 * @brief Structure representing the encoder commands.
 *
 */
struct EncoderCmd {
    std::vector<uint8_t> zero_cmd;   // 0:normal read; 1: set to zero
};

/**
 * @struct HeadLightCmd
 *
 * @brief Structure representing the head light command.
 */
struct HeadLightCmd {
    uint8_t r = 0;               // Red PWM duty
    uint8_t g = 0;               // Green PWM duty
    uint8_t b = 0;               // Blue PWM duty
    uint8_t brightness = 0;      // Overall brightness
    uint8_t on_ratio = 0xFF;     // On-time duty ratio
    uint8_t period_100ms = 0;    // Effect period in 100 ms units
};

/**
 * @struct HeadCmd
 *
 * @brief Structure representing the head commands.
 *
 * This structure holds servo commands, light strip data, calibration commands, and self-test commands.
 *
 */
struct HeadCmd {
    std::vector<double> q;               // Normalized servo positions [0, 100]
    std::vector<uint16_t> dq;            // Desired servo velocities
    std::vector<uint16_t> ddq;           // Desired servo accelerations
    std::vector<uint16_t> tau;           // Desired servo torques
    HeadLightCmd light;                   // Light color and timing control
    uint8_t command;                     // 0xFA/0xFC/0xFD/0xFE/0xF9
    uint8_t servo_id;                    // 0=left, 1=right, used by 0xFD
    uint8_t direction;                   // 0=increase, 1=decrease
    uint8_t save_flag;                   // 0=step, 1=save to EEPROM
};

/**
 * @struct HeadState
 *
 * @brief Structure representing the head state.
 *
 * This structure holds servo positions, peripheral states, and communication status.
 *
 */
struct HeadState {
    std::vector<double> q;               // Normalized servo positions [0, 100]
    std::vector<int16_t> dq;             // Current servo velocities
    std::vector<int16_t> tau;            // Current servo torques
    std::vector<uint8_t> servo_online;   // Online states of two servos
    uint8_t key_state;                   // Raw key state
    uint16_t light_intensity;            // Light intensity
    uint8_t online_status;               // 0=online, 1=offline
    uint16_t error_code;                 // Communication error code
};

/**
 * @struct MotorCmd
 *
 * @brief Structure representing the motor commands.
 *
 * This structure holds the desired joint angles , the desired joint velocities , the desired torque , the proportional gains, the derivative gains, the integral gains, and the names of the motors.
 *
 */
struct MotorCmd {
  std::vector<double> q;           // The desired angles.
  std::vector<double> dq;          // The desired velocities.
  std::vector<double> tau;         // The desired output torque.
  std::vector<double> Kp;          // Proportional gains
  std::vector<double> Kd;          // Derivative gains
  std::vector<double> Ki;          // Integral gains
  std::vector<int> control_mode;   // Control mode
  std::vector<std::string> names;  // The names of the motors
};

/**
 * @struct MotorState
 *
 * @brief Structure representing the state of the motors.
 *
 * This structure holds the current joint angles , the current joint velocities , the current estimated output torque , the current proportional gains, the current derivative gains, and the current integral gains.
 *
 */
struct MotorState {
  std::vector<double> q;                // The current angles
  std::vector<double> dq;               // The current velocities
  std::vector<double> tau;              // The current estimated output torque
  std::vector<double> Kp;               // The current proportional gains
  std::vector<double> Kd;               // The current derivative gains
  std::vector<double> Ki;               // The current integral gains
  std::vector<double> motor_T;          // The current motor temperature
  std::vector<double> mos_T;            // The current MOS temperature
  std::vector<double> current;          // The current of the motor
  std::vector<uint16_t> loss_num;       // Loss count
  std::vector<uint32_t> status_code;    // Status code
  std::vector<uint32_t> error_code;     //  Error code
  std::vector<uint32_t> reserve;        // Reserved for future use
};

/**
 * @struct BmsCmd
 *
 * @brief Structure representing the command for controlling a Battery Management System (BMS).
 *
 * This structure holds the command to switch the BMS on or off, along with reserved bytes for future use.
 *
 */
struct BmsCmd {
  std::vector<uint8_t> switch_cmds;  // 1 turn on; 0 turn off;
  std::vector<uint32_t> reserve;
};

/**
 * @struct BmsState
 *
 * @brief Structure representing the state of a Battery Management System (BMS).
 *
 * This structure holds various states of a BMS, including its status, state of charge (SOC), 
 * voltage, current, temperature, cycle count, battery quantity, capacity information, and production date.
 *
 */
struct BmsState {
  uint8_t bms_status;    // discharging , charing, over voltage etc.
  double SOC;           // SOC 0-100%
  double voltage;       // The battery voltage (unit: V)
  double current;       // （unit: A)
  double temperature;  // The battery temperature (unit: °C)
  uint16_t cycle;        // The current number of cycles of the battery
  uint16_t battery_count;    // The number of battery cells
  uint32_t remaining_capacity; // Remaining capacity of the battery (unit: mAh)
  uint32_t nominal_capacity;   // Nominal capacity of the battery (unit: mAh)
  uint16_t production_year;    // Production year of the battery
  uint8_t production_month;    // Production month of the battery (1-12)
  uint8_t production_day;      // Production day of the battery (1-31)
};

/**
 * @struct HandCmd-->EECmd todo
 *
 * @brief Structure representing the command for controlling a dexterous hand.
 *
 * This structure holds the desired position, desired velocities, desired torque, and control mode for the hand.
 *
 */
struct EECmd {
  std::vector<double> q;      // The desired position.
  std::vector<double> dq;     // The desired velocities.
  std::vector<double> tau;    // The desired torque.
  std::vector<double> Kp;     // Proportional gains
  std::vector<double> Kd;     // Derivative gains
  std::vector<double> Ki;     // Integral gains
  std::vector<int16_t> mode;  // The control mode.(Such as joint position control, joint torque control etc.)
};

/**
 * @struct HandState -->EEState todo
 *
 * @brief The state of the hand.
 *
 * This structure holds the current angles, current force, current current, current code, and reserved data for future use.
 *
 */
struct EEState {
  std::vector<double> q;        // The current angles (unit:such as in radians)
  std::vector<double> dq;       // The current velocities (unit:such as in radians/s)
  std::vector<double> tau;    // The current force (unit: such as in Newton)
  std::vector<double> current;  // The current current (unit:such as in mA)
  std::vector<double> Kp;       // Proportional gains
  std::vector<double> Kd;       // Derivative gains
  std::vector<double> Ki;       // Integral gains
  std::vector<int16_t> code;    // The current code
  std::vector<double> reserve;  // Reserved for future use(tactile feedback, etc.)
};

struct RemoteState {
  std::vector<uint16_t> sbus_ch; 
};

struct UltrasonicSensorState {
  std::vector<uint16_t> distance; 
};

struct TemperatureSensorState {
  std::vector<double> temperature; 
};

struct AlarmState {
  std::vector<uint32_t> alarm_code; 
};

/**
 * @struct RobotState
 *
 * @brief Structure representing the state of a robot.
 *
 * This structure holds various states of a robot, including IMU data, motor states, head states, hand states, force sensor states, and battery management system (BMS) states.
 */
struct RobotState {
  int64_t timestamp;                     // Timestamp in nanoseconds, typically represents the time when this data was recorded or generated.
  std::vector<ImuState> imu_state;  // IMU state for multiple IMUs

  MotorState motor_states;  // Arms, Legs/wheels, torso
  
  HeadState head_state;  // Head board

  EEState hand_states;   // Dexterous hand

  EncoderState encoder_states; // Encoder states

  ForceSensorState force_states;  // The current force sensor states;

  TorsoForceState torso_states;

  RemoteState remote_state;

  UltrasonicSensorState ultrasonic_state;

  TemperatureSensorState temperature_state;

  std::vector<BmsState> bms_state;  // IMU state for multiple BMSs

  AlarmState alarm_states;  // Alarm states

  std::vector<double> reserve;  // Reserved for future use(tactile feedback, etc.)
};

/**
 * @struct JoystickState
 *
 * @brief Structure representing sensor inputs from robot joystick.
 *
 * This structure contains timestamp information along with axis and button values obtained from a joystick sensor.
 */
struct JoystickState {
  int64_t timestamp;                // Timestamp in nanoseconds, associated with the sensor input.
  std::vector<float> axes;       // Values representing the positions of the joystick axes. Left stick x,y; Right stick x,y;
  std::vector<int32_t> buttons;  // Values representing the state of joystick buttons.
};

/**
 * @struct RobotCmd
 *
 * @brief Structure representing the command for controlling a robot.
 *
 * This structure holds various commands that can be used to control a robot, including the desired angles, desired velocities, desired output torque, desired position stiffness, and desired velocity stiffness etc.
 */
struct RobotCmd {
  int64_t timestamp;                // Timestamp in nanoseconds, typically represents the time when this data was recorded or generated.
  MotorCmd motor_cmds;  // Arms, Legs/wheels
  EncoderCmd encoder_cmds;  // Encoder
};

/**
 * @struct DiagnosticComponent
 *
 * @brief Structure representing the status of a single robot component.
 *
 * This structure holds the module ID, device ID, and status code for one component.
 */
struct DiagnosticComponent {
  uint32_t module_id;  // Module ID
  uint32_t device_id;  // Device ID in the module
  uint32_t status;     // Status code, with bits representing different status flags.
};

/**
 * @struct DiagnosticStatus
 *
 * @brief Structure representing the status of multiple robot components with a common timestamp.
 *
 * This structure holds the timestamp and a collection of component statuses.
 */
struct DiagnosticStatus {
  int64_t timestamp;      // Common timestamp in nanoseconds for all components
  std::vector<DiagnosticComponent> components;  // Statuses for multiple components
};


typedef std::shared_ptr<RobotCmd> RobotCmdPtr;

typedef std::shared_ptr<RobotState> RobotStatePtr;

typedef std::shared_ptr<HeadState> HeadStatePtr;

typedef std::shared_ptr<BmsState> BmsStatePtr;

typedef std::shared_ptr<EEState> EEStatePtr;

typedef std::shared_ptr<AlarmState> AlarmStatePtr;

typedef std::shared_ptr<JoystickState> JoystickStatePtr;

typedef std::shared_ptr<DiagnosticStatus> DiagnosticStatusPtr;
#endif