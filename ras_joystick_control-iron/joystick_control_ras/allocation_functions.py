"""
This file contains the allocation functions for the different vessels
"""
import pygame

def joy2act_TN_01(joy: pygame.joystick.Joystick):
    """
    joy2act_TN_01 Convert joystick input to Tito Neri Vessel actuation
    Bart Boogmans 2024

    u = [rpm_PS_thr,rpm_SB_thr,pwm_bow,alpha_PS_azi,alpha_SB_azi]
 
    From top view, positive azimuth rotations are clockwise with a zero
    angle resulting in forward thrust at positive thrust rps.
    Bow thrust assumed to point in +y (SB) direction at positive PWM output
    
    Axis/button mapping:
    Axis 0: main stick horizontal (left/right)
    Axis 1: main stick vertical (forward/backward)
    Axis 2: main stick rotation (counter clockwise/clockwise)
    Axis 3: small flipper

    Button 0: big pew pew
    Button 1: as numbered (1)
    Button 2: as numbered (2)
    Button 3: ...
    
    """

    ax_speed = (-joy.get_axis(3))/2.0 +0.5

    if ax_speed > 0.05:
        speed = ax_speed
    else:
        speed = -joy.get_axis(1)

    # if abs(speed) <0.02: set to zero
    if abs(speed) < 0.02:
        speed = 0.0
        
    speed_TN_aft_max = 3000
    rotation_azi_TN_set_max = (1/3)*3.14159
    bow_TN_set_max = 0.3

    return [    speed_TN_aft_max*speed, 
                speed_TN_aft_max*speed,
                (joy.get_button(3)-joy.get_button(2))*bow_TN_set_max*(1+joy.get_button(0)),
                -rotation_azi_TN_set_max*joy.get_axis(0), 
                -rotation_azi_TN_set_max*joy.get_axis(0)]