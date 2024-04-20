//
// Created by colin.barker on 02/02/2024.
//
//

#ifndef APP_SMARTPUMP_REGISTERS_H
#define APP_SMARTPUMP_REGISTERS_H

// -----------------------------------------------------------------------------
// Useful values
// -----------------------------------------------------------------------------
#define SOURCE_SETVAL             (0)
#define SOURCE_ANA1               (1)
#define SOURCE_ANA2               (2)
#define SOURCE_ANA3               (3)
#define SOURCE_FLOW               (4)
#define SOURCE_DIGITAL_PRESSURE   (5)

#define MODE_MANUAL    (0)
#define MODE_PID       (1)
#define MODE_BANGBANG  (2)

#define DEVICE_TYPE_GP  (2)
#define DEVICE_TYPE_SPM (3)
//***********************************************************************************
//* Registers list below
//***********************************************************************************

// -----------------------------------------------------------------------------
// General settings
// -----------------------------------------------------------------------------
#define REGISTER_PUMP_ENABLE             (0)
#define REGISTER_POWER_LIMIT_MILLIWATTS  (1)

#define REGISTER_STREAM_MODE_ENABLE      (2)


// -----------------------------------------------------------------------------
// Measurements
// -----------------------------------------------------------------------------
#define REGISTER_MEAS_DRIVE_VOLTS        (3)
#define REGISTER_MEAS_DRIVE_MILLIAMPS    (4)
#define REGISTER_MEAS_DRIVE_MILLIWATTS   (5)
#define REGISTER_MEAS_DRIVE_FREQ         (6)
#define REGISTER_MEAS_ANA_1              (7)
#define REGISTER_MEAS_ANA_2              (8)
#define REGISTER_MEAS_ANA_3              (9)
#define REGISTER_MEAS_FLOW               (32)
#define REGISTER_MEAS_DIGITAL_PRESSURE   (39)
#define REGISTER_MEAS_DRIVE_PHASE        (41)


// -----------------------------------------------------------------------------
// Measurement settings
// -----------------------------------------------------------------------------
#define REGISTER_SET_VAL         (23)
#define REGISTER_ANA_1_OFFSET    (24)
#define REGISTER_ANA_1_GAIN      (25)
#define REGISTER_ANA_2_OFFSET    (26)
#define REGISTER_ANA_2_GAIN      (27)
#define REGISTER_ANA_3_OFFSET    (28)
#define REGISTER_ANA_3_GAIN      (29)
#define REGISTER_DIGITAL_PRESSURE_OFFSET (40)


// -----------------------------------------------------------------------------
// Control settings
// -----------------------------------------------------------------------------

// Control mode
#define REGISTER_CONTROL_MODE        (10)

// Manual control settings
#define REGISTER_MANUAL_MODE_SETPOINT_SOURCE  (11)

// PID control settings
#define REGISTER_PID_MODE_SETPOINT_SOURCE  (12)
#define REGISTER_PID_MODE_MEAS_SOURCE      (13)
#define REGISTER_PID_PROPORTIONAL_COEFF    (14)
#define REGISTER_PID_INTEGRAL_COEFF        (15)
#define REGISTER_PID_INTEGRAL_LIMIT_COEFF  (16)
#define REGISTER_PID_DIFFERENTIAL_COEFF    (17)
#define REGISTER_RESET_PID_ON_TURNON       (33)

// Bang bang control settings
#define REGISTER_BANG_BANG_MEAS_SOURCE             (18)
#define REGISTER_BANG_BANG_LOWER_THRESH            (19)
#define REGISTER_BANG_BANG_UPPER_THRESH            (20)
#define REGISTER_BANG_BANG_LOWER_POWER_MILLIWATTS  (21)
#define REGISTER_BANG_BANG_UPPER_POWER_MILLIWATTS  (22)


// -----------------------------------------------------------------------------
// Miscellaneous settings
// -----------------------------------------------------------------------------
#define REGISTER_STORE_CURRENT_SETTINGS  (30)

#define REGISTER_ERROR_CODE              (31)

#define REGISTER_USE_FREQUENCY_TRACKING  (34)
#define REGISTER_MANUAL_DRIVE_FREQUENCY  (35)

#define REGISTER_FIRMWARE_VERSION        (36)
#define REGISTER_DEVICE_TYPE             (37)
#define REGISTER_FIRMWARE_MINOR_VERSION  (38)


// -----------------------------------------------------------------------------
// Communication settings
// -----------------------------------------------------------------------------
#define REGISTER_DRIVER_I2C_ADDRESS 		  (42)
#define REGISTER_COMMUNICATION_INTERFACE 	(43)

#endif //APP_SMARTPUMP_REGISTERS_H
