import math as m
#max_tics is the maximum number of tics per second the encoder can read. This parameter depends on which microcontroller is used
#reduction is the motor to wheels reduction. This paramter depends on the physical structure of the robot
#encoder is the encoder resolution. This parameter depends on which motor brand and model is used
def motor_voltage_to_wheels_speed(motorA,motorB, max_tics=(700/0.01),reduction=(1./3.),encoder=512.*19.):
    motorA_rad_per_s = (motorA/255.0)*(max_tics/encoder)*m.pi;
    wheelA = motorA_rad_per_s*reduction;
    motorB_rad_per_s = (motorB/255.0)*(max_tics/encoder)*m.pi;
    wheelB = motorB_rad_per_s*reduction;
    return wheelA, wheelB
#wheel_radius is the wheel radius. It depends on the robot
#robot_lenght is the distance from one wheel to the other. It depends on the robot
def wheels_speeds_to_robots_speeds(wheelA,wheelB,wheel_radius=0.03,robot_lenght=0.075):
    angular_speed = wheel_radius*(wheelA - wheelB)/robot_lenght;
    linear_speed = wheel_radius*(wheelA + wheelB)/2;
    return angular_speed, linear_speed
