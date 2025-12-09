#!/usr/bin/env python3

"""
Simple PID controller implementation for drone attitude and position control.
"""


class PIDController:
    """
    PID (Proportional-Integral-Derivative) controller for drone control.
    """

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, min_output=-1.0, max_output=1.0):
        """
        Initialize PID controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            min_output: Minimum output value
            max_output: Maximum output value
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

    def reset(self):
        """Reset PID controller state"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

    def compute(self, setpoint, current_value, current_time):
        """
        Compute PID control output.
        
        Args:
            setpoint: Desired value
            current_value: Current measured value
            current_time: Current time in seconds
            
        Returns:
            Control output
        """
        error = setpoint - current_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        if self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                self.integral += error * dt
                # Anti-windup: limit integral term
                max_integral = (self.max_output - self.min_output) / max(self.ki, 1e-6)
                self.integral = max(-max_integral, min(max_integral, self.integral))
        
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = 0.0
        if self.last_time is not None and current_time > self.last_time:
            dt = current_time - self.last_time
            if dt > 0:
                error_rate = (error - self.last_error) / dt
                d_term = self.kd * error_rate
        
        # Compute output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(self.min_output, min(self.max_output, output))
        
        # Update state
        self.last_error = error
        self.last_time = current_time
        
        return output


