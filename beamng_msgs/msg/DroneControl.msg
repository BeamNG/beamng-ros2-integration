# Drone control message for BeamNG drone vehicle
# All values are normalized to [-1.0, 1.0] range unless otherwise specified

float64 ascend          # Vertical ascent input [0.0, 1.0]
float64 descend         # Vertical descent input [0.0, 1.0]
float64 pitch           # Pitch control (nose up/down) [-1.0, 1.0]
float64 roll            # Roll control (tilt left/right) [-1.0, 1.0]
float64 yaw             # Yaw control (rotate left/right) [-1.0, 1.0]
float64 yaw_left        # Yaw left input [0.0, 1.0]
float64 yaw_right       # Yaw right input [0.0, 1.0]
bool takeoff_land       # Toggle takeoff/land (true = takeoff, false = land)
bool toggle_mode        # Toggle flight mode (true = switch mode)
