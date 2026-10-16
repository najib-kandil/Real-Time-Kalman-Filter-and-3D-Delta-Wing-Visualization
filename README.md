# Real-Time-Kalman-Filter-and-3D-Delta-Wing-Visualization
This project implements a real-time sensor acquisition and filtering system using an Arduino Nano, an MPU6050 IMU, and MATLAB.
The system reads accelerometer and gyroscope measurements from the MPU6050, processes the data using a discrete-time Kalman predictor/filter, and displays the results in real time.

A 3D delta-wing drone model is also visualized in MATLAB. Its orientation is updated using a complementary filter based on the IMU measurements.
Main Features:

-Real-time accelerometer and gyroscope data acquisition.
-Arduino Nano communication with MATLAB through USB.
-MPU6050 communication using the I2C protocol.
-Discrete-time Kalman predictor and filter.
-Live plotting of raw and filtered sensor outputs.
-Real-time 3D delta-wing visualization.
-Roll, pitch, and yaw estimation.
-Adjustable sampling frequency and filter parameters.

2. Hardware Requirements

-Arduino Nano compatible with the MATLAB nano3 board configuration.
-MPU6050 six-axis inertial measurement unit (IMU).
-USB cable for Arduino programming and communication.
-Jumper wires.
-Computer running MATLAB.

3. Arduino and MPU6050 Connections

The MPU6050 communicates with the Arduino Nano through the I2C interface.

Wiring:
VCC ---> 3.3 V or 5 V, as supported by the specific module
GND ---> GND
SDA ---> A4
SCL ---> A5

Important: Verify the voltage requirements of your particular MPU6050 breakout board before connecting VCC. Some modules include a voltage regulator and level shifting, while others require a 3.3 V supply and logic levels.

Connect the Arduino Nano to the computer through USB. In this project, MATLAB communicates with the board using the COM7 serial port configuration.

If your computer assigns a different COM port, update the port name in the MATLAB script.

4. Software Requirements

-MATLAB.
-MATLAB Support Package for Arduino Hardware.
-The required Arduino board support for the Nano configuration used by the script.
-MPU6050 support compatible with the MATLAB mpu6050 interface.
-The I2C library enabled in the Arduino connection.

The project uses the following MATLAB commands to establish communication:

a = arduino('COM7', 'nano3', 'Libraries', 'I2C');

fs = 100;

imu = mpu6050(a, ...
    'SampleRate', fs, ...
    'OutputFormat', 'matrix');

The requested sampling frequency is 100 Hz, corresponding to a nominal sampling interval of:

[
\Delta t = \frac{1}{f_s} = 0.01\ \text{s}
]

The actual acquisition and processing timing may vary depending on hardware communication and MATLAB execution overhead.

5. Kalman Filter Description

The MATLAB implementation uses a six-state discrete-time model:

[
\mathbf{x}_k =
\begin{bmatrix}
x_1 & x_2 & x_3 & v_1 & v_2 & v_3
\end{bmatrix}^{T}
]

The first three states are position-like variables, while the last three are velocity-like variables.

The state transition matrix is:

[
A =
\begin{bmatrix}
I_3 & \Delta t I_3 \
0_3 & I_3
\end{bmatrix}
]

The input matrix is:

[
B =
\begin{bmatrix}
0_3 \
\Delta t I_3
\end{bmatrix}
]

The measurement model is:

[
y_k = C\mathbf{x}_k + v_k
]

where:

[
C = \begin{bmatrix}1&1&1&0&0&0\end{bmatrix}
]

In the implementation, the measured scalar is constructed as the sum of the three accelerometer channels:

[
y_k = a_x + a_y + a_z
]

The predictor and filter use the configured process and measurement noise covariance matrices to update the state estimate and its uncertainty.

Note: This is the project's implemented mathematical model. The three position-like states should not be interpreted as validated physical positions, because the measurement equation uses summed acceleration rather than direct position measurements. The current implementation is not a conventional, fully calibrated inertial navigation filter.

6. Predictor and Filter Equations

The state predictor is implemented as:

[
\hat{\mathbf{x}}_{k+1}^{-}
= A\hat{\mathbf{x}}_k^{-}

B\mathbf{u}_k

K_k e_k
]

The measurement innovation is:

[
e_k = y_k - C\hat{\mathbf{x}}_k^{-}
]

The dynamic gain is calculated using the covariance matrices:

[
K_k =
(AP_kC^T+V_{12})
(CP_kC^T+V_2)^{-1}
]

The covariance update is:

[
P_{k+1} =
AP_kA^T + V_1

K_k(CP_kC^T+V_2)K_k^T
]

A separate filter gain is used to correct the current predicted state:

[
K_{0,k}=P_kC^T(CP_kC^T+V_2)^{-1}
]

[
\hat{\mathbf{x}}_k^f =
\hat{\mathbf{x}}k^- + K{0,k}e_k
]

The filtered scalar output is calculated as:

[
y_k^f=C\hat{\mathbf{x}}_k^f
]

The matrices (V_1), (V_2), and (V_{12}) define the configured uncertainty model. Their values should be validated against the actual sensor units, noise characteristics, and intended physical model before interpreting the results quantitatively.

7. 3D Delta-Wing Visualization

The MATLAB script creates a simplified 3D delta-wing aircraft using graphical objects and an hgtransform transformation.

The orientation is estimated separately from the six-state Kalman filter using a complementary filter.

The accelerometer provides a gravity-based reference for roll and pitch, while the gyroscope supplies angular-rate measurements.

The complementary filter combines these sources:

[
\phi_k =
\alpha(\phi_{k-1}+\omega_x\Delta t)
+(1-\alpha)\phi_{\mathrm{acc}}
]

[
\theta_k =
\alpha(\theta_{k-1}+\omega_y\Delta t)
+(1-\alpha)\theta_{\mathrm{acc}}
]

Yaw is obtained by integrating the gyroscope's vertical-axis angular rate.

The rotation matrix is constructed using Z-Y-X Euler rotations:

[
R=R_z(\psi)R_y(\theta)R_x(\phi)
]

where:

(\phi): roll angle.

(\theta): pitch angle.

(\psi): yaw angle.

The 3D model is updated using the resulting rotation matrix.

Limitations: The MPU6050 has no magnetometer, so yaw will drift over time. Accelerometer-based roll and pitch estimates are also affected by linear acceleration, sensor mounting orientation, and measurement noise. The current visualization estimates attitude only; it does not estimate the drone's translational position or reconstruct a physical flight trajectory.

8. Running the Project

1. Connect the MPU6050 to the Arduino Nano according to the wiring table.
2. Connect the Arduino Nano to the computer using USB.
3. Confirm the board's assigned COM port in MATLAB or your operating system.
4. Verify that the required Arduino and IMU support packages are installed.
5. Open the MATLAB script.
6. Change COM7 if your Arduino uses a different port.
7. Run the script.
8. Observe the raw and filtered output plot and the 3D delta-wing visualization.
9. Move or rotate the sensor to observe the changes in the estimated orientation.
10. Stop execution with Ctrl+C in the MATLAB Command Window.

9. Troubleshooting

-Arduino connection error
-Verify the COM port.
-Ensure that no other application is using the port.
-Check the USB cable and board connection.

Confirm the board identifier and support-package compatibility.

MPU6050 communication error

-Check VCC and GND connections.
-Verify SDA and SCL wiring.
-Confirm I2C compatibility and the sensor's address/configuration.
-Ensure the installed MATLAB IMU interface supports the connected device.

Incorrect or unstable attitude

-Verify accelerometer and gyroscope units.
-Confirm that the gyroscope measurements are in radians per second before integration.
-Check the sensor-to-aircraft axis alignment.
-Calibrate gyroscope bias and accelerometer offsets.
-Adjust the complementary-filter coefficient only after verifying the measurements.

Noisy or unexpected Kalman output

-Verify the measurement definition and state-space model.
-Check the dimensions and units of all covariance matrices
-Tune the noise covariance matrices using measured sensor data.
-Confirm that the accelerometer measurement model is physically consistent with the state variables being estimated.

10. Project Limitations and Future Improvements

Possible improvements include:

-Calibrating the MPU6050 before operation.
-Implementing a conventional inertial attitude estimator, such as an extended Kalman filter or a quaternion-based complementary filter.
-Adding a magnetometer for heading correction.
-Replacing the summed-acceleration measurement with a measurement model appropriate to the estimated states.
-Implementing gravity compensation and bias estimation.
-Adding velocity and position estimation with suitable drift mitigation.
-Improving the 3D aircraft geometry and adding flight-path visualization.
-Logging sensor measurements and estimated states for offline analysis.

11. Summary

This project demonstrates the integration of Arduino-based IMU acquisition with MATLAB signal processing, a discrete-time Kalman predictor/filter, and real-time 3D aircraft attitude visualization.

It provides a foundation for investigating sensor fusion, estimation algorithms, and visualization techniques for a delta-wing drone.

The current implementation is intended for experimentation and educational development. Its state estimates and attitude outputs should be validated before use in flight-control or navigation applications.
