import tkinter as tk
from tkinter import ttk
import threading
import time

# Simulated variables (as if interacting with Arduino) for testing reasons
motor_running = False
motor_speed = 0  # The target speed to be maintained by PID
current_speed = 0  # The actual speed (simulated motor speed)
set_speed_value = 0  # Desired motor speed from the slider

# PID constants 
kp = 3.0  # Proportional gain
ki = 0.3  # Integral gain
kd = 0.1  # Derivative gain

previous_error = 0  # Store the last error for the derivative term
integral = 0  # Accumulated error for the integral term

# Exponential smoothing factor
alpha = 0.3  # Smoothing factor (0 < alpha <= 1)
smoothed_speed = 0  # Smoothed speed

# Functions to simulate PID-controlled motor
def pid_control():
    global motor_running, current_speed, set_speed_value, previous_error, integral, motor_speed, smoothed_speed

    while True:
        if motor_running:
            # Apply exponential smoothing to set_speed_value
            smoothed_speed = alpha * set_speed_value + (1 - alpha) * smoothed_speed

            # Calculate the error
            error = smoothed_speed - current_speed

            # Proportional term
            p_term = kp * error

            # Integral term
            integral += error
            i_term = ki * integral

            # Derivative term
            derivative = error - previous_error
            d_term = kd * derivative

            # PID output (motor speed control)
            motor_speed = p_term + i_term + d_term

            # Limit motor speed between 0 and 255 (as in a real system)
            motor_speed = max(0, min(255, motor_speed))

            # Update current speed gradually (simulating motor response)
            current_speed += (motor_speed - current_speed) * 0.1  # Smooth transition toward motor_speed

            # Update previous error for the next cycle
            previous_error = error
        else:
            current_speed = 0  # Reset the speed when motor is stopped

        time.sleep(0.1)  # Simulate motor update rate every 100ms

# GUI functions to interact with the simulated motor
def start_motor():
    global motor_running
    motor_running = True

def stop_motor():
    global motor_running, set_speed_value
    motor_running = False
    set_speed_value = 0  # Reset the desired speed when stopping the motor

def set_speed(val):
    global set_speed_value
    set_speed_value = float(val)  # Set the desired speed from the slider

def update_speed_label():
    current_speed_var.set(f"Current Speed: {current_speed:.2f} RPM")
    root.after(100, update_speed_label)  # Check every 100 ms to update the display

# Start the motor simulation with PID control in a separate thread
pid_thread = threading.Thread(target=pid_control, daemon=True)
pid_thread.start()

# Setup the GUI
root = tk.Tk()
root.title("Motor Control with PID Simulation")
root.geometry("400x300")  # Set window size
root.config(bg="#dce5fa")  # Background color

button_frame = tk.Frame(root, bg="#0f0f0f")
button_frame.pack(pady=10)

# UI Elements
start_button = tk.Button(root, text="Start Motor", command=start_motor, bg="#4CAF50", fg="white", font=("Arial", 12))
start_button.pack(pady=10)

stop_button = tk.Button(root, text="Stop Motor", command=stop_motor, bg="#f44336", fg="white", font=("Arial", 12))
stop_button.pack(pady=10)

speed_slider = tk.Scale(root, from_=0, to=255, orient=tk.HORIZONTAL, label="Set Speed (0-255)", command=set_speed)
speed_slider.pack(pady=10)

current_speed_var = tk.StringVar()
current_speed_var.set("Current Speed: 0 RPM")
speed_label = ttk.Label(root, textvariable=current_speed_var)
speed_label.pack(pady=10)

# Start checking for speed updates
update_speed_label()

root.mainloop()
