/*
 * PID.c
 *
 *  Created on: Nov. 15, 2023
 *      Author: ahuja
 */

#include "tcs34725.h"
#include "L298N.h"

// Define PID constants
#define KP 1.2
#define KI 1.0
#define KD 0.1

// Define target values (adjust as needed)
#define TARGET_VALUE 5000
#define TARGET_SPEED 50

// PID Controller structure
typedef struct {
    float previous_error;
    float integral;
} PID_Controller;

// Initialize PID controller
void PID_Init(PID_Controller pid) {
    pid->previous_error = 0;
    pid->integral = 0;
}

// PID control function
int PID_Control(float sensor_value, PID_Controllerpid) {
    float error = TARGET_VALUE - sensor_value;

    // PID terms
    float proportional = KP * error;
    float integral = KI * (error + pid->integral);
    float derivative = KD * (error - pid->previous_error);

    // Calculate control signal
    int control_signal = (int)(proportional + integral + derivative);

    // Update PID values for next iteration
    pid->previous_error = error;
    pid->integral += error;

    return control_signal;
}

// Main function
int main() {
    // Initialize components and PID controller
    TCS34725_HandleTypeDef sensor1, sensor2;
    L298N_HandleTypeDef motor;
    PID_Controller pid;
    PID_Init(&pid);

    // Initialize sensors and motors (assuming they are already configured)

    while(1) {
        // Read sensor values
        tcs34725_get_data(&sensor1, &hi2c_device1);
        tcs34725_get_data(&sensor2, &hi2c_device2);

        // Calculate the average sensor value from both sensors
        float sensor_value = (sensor1.red + sensor2.red) / 2; // Adjust this based on your sensor readings

        // Perform PID control to get motor speeds
        int control_signal = PID_Control(sensor_value, &pid);

        // Adjust motor speeds based on control signal
        int left_speed = TARGET_SPEED + control_signal;
        int right_speed = TARGET_SPEED - control_signal;

        // Apply control to motors
        l298n_drive_forward(&motor, &timer, left_speed, right_speed);
    }
    return 0;
}
