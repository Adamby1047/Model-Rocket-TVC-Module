## Thrust Vector Control Module for Model Rocketry
![TVC render3](https://github.com/Adamby1047/Model-Rocket-TVC-Module/assets/74665879/a42e5fd5-dd39-4b41-9983-5475d4c06f0f) ![TVC render4](https://github.com/Adamby1047/Model-Rocket-TVC-Module/assets/74665879/04e2af0b-a9b8-4d4e-b734-0965829d649c)
### Project Description
This thrust vector control mount was designed to fit an apogee e6 medalist model rocket motor, with a diameter of ~24 mm. The module utilizes two cheap servo motors to create a 2-axis gimbal with 8 degrees of freedom across each axis. Orientation is measured using the BNO055 9-axis IMU absolute orientation, however, for a real test flight this internal sensor fusion will not be viable - raw gyroscope/accelerometer data must be used along with a filter to obtain accurate orientation (relative to a starting position...likely on the launch pad). The system is controlled by a teensy 4.1, programmed in C++. 

First static test fire: https://www.youtube.com/watch?v=-P_c4bo75B4&ab_channel=AdamBenYoussef
PID Controller testing: https://www.youtube.com/watch?v=Zy2BaQEeLKs&ab_channel=AdamBenYoussef

The PID controller created is a rough draft outlining the main logic that needs to be set for the control system. Clamping on the integral term, and a low pass filter on the derivative term need to be implemented. However this is a good base going forward. The controller also needs to be tuned to the specific rocket parameters.

