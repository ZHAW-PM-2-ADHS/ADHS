#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include <Eigen/Dense>
#include "ColorSensor.h"
#include "DCMotor.h"
#include "DebounceIn.h"
#include "SensorBar.h"

#define M_PIf 3.14159265358979323846f // pi

float speed_filtered_reset = 1.0f;

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end

/**
 *
 * @param motor_M1 Motor 1
 * @param motor_M2 Motor 2
 * @param active_sensor_bar Sensor bar to be used
 * @param direction Driving Direction (1 = forward, -1 = backward)
 * @param angle Angle measured by the line follower
 * @param Cwheel2robot
 * @param wheel_vel_max Max velocity
 * @param r_wheel wheel radius
 */
void driveRobot(DCMotor &motor_M1,
                DCMotor &motor_M2,
                float &direction,
                float &angle,
                const Eigen::Matrix2f &Cwheel2robot,
                float wheel_vel_max,
                float r_wheel,
                float speed);

void stopRobot(DCMotor &motor_M1, DCMotor &motor_M2);

int detectLine(SensorBar *&sensor_bar);

void measureColor(
    ColorSensor &sensor, float raw[4], float avg[4], float cal[4], int &color_num, const char *&color_string);

// main runs as an own thread
int main()
{
    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(LED1);

    // additional led
    // create DigitalOut object to command extra led, you need to add an additional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
    DigitalOut led1(PB_9);

    // --- adding variables and objects and applying functions starts here ---

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    constexpr float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
    // 6.0f V if you only use one battery pack

    constexpr float b_wheel = 0.128f;         // wheelbase, distance from wheel to wheel in meters
    constexpr float bar_dist = 0.07f;         // distance from wheel axis to leds on sensor bar / array in meters
    constexpr float d_wheel = 0.0596f;        // wheel diameter in meters
    constexpr float r_wheel = d_wheel / 2.0f; // wheel radius in meters

    // https://www.pololu.com/product/3490/specs
    constexpr float gear_ratio = 100.00f;
    constexpr float kn = 140.0f / 12.0f;

    float speed = 0.5f;

    // motor M1 and M2, do NOT enable motion planner when used with the LineFollower (disabled per default)
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    // transforms wheel to robot velocities
    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot << r_wheel / 2.0f, r_wheel / 2.0f, r_wheel / b_wheel, -r_wheel / b_wheel;

    float sensor_direction = 1.0f;

    // Both sensor bars, one in front, one at the back
    SensorBar sensor_bar_1(PB_9, PB_8, bar_dist);
    SensorBar sensor_bar_2(PB_3, PB_10, bar_dist);

    // Pointer to the sensor bar actually being used
    SensorBar *sensor_bar_front = &sensor_bar_1;
    SensorBar *sensor_bar_back = &sensor_bar_2;

    // angle measured from sensor bar (black line) relative to robot
    float angle{0.0f};

    // rotational velocity controller
    const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();

    // color sensor
    float color_raw_Hz[4] = {
        0.0f, 0.0f, 0.0f, 0.0f}; // define an array to store the measurement of the color sensor (in Hz)
    float color_avg_Hz[4] = {
        0.0f, 0.0f, 0.0f, 0.0f}; // define an array to store the average measurement of the color sensor (in Hz)
    float color_cal[4] = {
        0.0f, 0.0f, 0.0f, 0.0f}; // define an array to store the calibrated measurement of the color sensor

    int color_num =
        0.0f; // define a variable to store the color number, e.g. 0 for red, 1 for green, 2 for blue, 3 for clear
    const char *color_string;       // define a variable to store the color string, e.g. "red", "green", "blue", "clear"
    ColorSensor Color_Sensor(PA_5); // create ColorSensor object, connect the frequency output pin of the sensor to PC_2

    // Calibration Values
    constexpr float black[4] = {381.5f, 353.5f, 409.0f, 1230.0f};
    constexpr float white[4] = {714.0f, 679.0f, 762.0f, 2230.0f};

    Color_Sensor.setCalibration(black, white);
    // start timer
    main_task_timer.start();

    // counter for timing specific events
    int cycle_counter = 0;

    // Variables to manage robot states outside of cycles
    bool skipping_line_detection = false;
    bool robot_stopped = false;
    bool just_started = true;

    // values to control the start of the robot
    constexpr float starting_angle = 0.2;
    constexpr int starting_cycle_count = 25;

    // Specific line follower readings (at the back) that help avoiding a stop at the crossing to the warehouse
    constexpr int backSensorReadingsToNotStop[] = {1, 3, 7, 12, 14, 15, 48, 96, 112, 128, 192, 224, 240};

    // this loop will run forever
    while (true) {

        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {
            // This condition is only for demonstration purposes.
            // Switching the direction based on time does not make sense
            /*if (cycle_counter % 500 == 0) {
                sensor_direction = -sensor_direction;

                if (sensor_bar_front == &sensor_bar_1) {
                    sensor_bar_front = &sensor_bar_2;
                    sensor_bar_back = &sensor_bar_1;
                } else {
                    sensor_bar_front = &sensor_bar_1;
                    sensor_bar_back = &sensor_bar_2;
                }
                cycle_counter++;
            }*/

            // angle = sensor_direction * sensor_bar_front->getAvgAngleRad();
            // printf("angle: %f\n", angle);
            // continue;

            if (!robot_stopped) {
                if (sensor_bar_front->isAnyLedActive() || just_started) {
                    int crossingLine = 0;
                    if (skipping_line_detection) {
                        cycle_counter++;
                        if (cycle_counter > 50) {
                            cycle_counter = 0;
                            skipping_line_detection = false;
                        }
                    } else {
                        crossingLine = detectLine(sensor_bar_front);
                    }

                    if (crossingLine == 0) {
                        enable_motors = 1;
                        if (just_started == true) {
                            angle = starting_angle;
                            cycle_counter++;
                            if (cycle_counter > starting_cycle_count) {
                                cycle_counter = 0;
                                just_started = false;
                            }
                        } else {
                            angle = sensor_direction * sensor_bar_front->getAvgAngleRad();
                        }
                        driveRobot(
                            motor_M1, motor_M2, sensor_direction, angle, Cwheel2robot, wheel_vel_max, r_wheel, speed);
                    } else {
                        stopRobot(motor_M1, motor_M2);
                        robot_stopped = true;
                        if (crossingLine == 1) {
                            printf("small line crossed");
                        } else if (crossingLine == 2) {
                            printf("big line crossed");
                        }
                    }
                }
            } else {
                int backSensorRaw = sensor_bar_back->getRaw();
                // printf("backsensorAVG: %d\n", backsensorAVG);
                if (std::find(std::begin(backSensorReadingsToNotStop),
                              std::end(backSensorReadingsToNotStop),
                              backSensorRaw) != std::end(backSensorReadingsToNotStop)) {
                    robot_stopped = false;
                } else {
                    enable_motors = 0;
                    cycle_counter++;

                    if (cycle_counter > 50) {
                        cycle_counter = 0;
                        robot_stopped = false;
                        skipping_line_detection = true;
                    }
                }
            }

            measureColor(Color_Sensor, color_raw_Hz, color_avg_Hz, color_cal, color_num, color_string);

            // Comment in if needed
            /*
            printf("Color Raw Hz: %f %f %f %f\n", color_raw_Hz[0], color_raw_Hz[1], color_raw_Hz[2], color_raw_Hz[3]);
            printf("Color Avg Hz: %f %f %f %f\n", color_avg_Hz[0], color_avg_Hz[1], color_avg_Hz[2], color_avg_Hz[3]);
            printf("Color Num: %d Color %s\n", color_num, color_string);
            */

        } else {

            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects

                for (int i = 0; i < 4; i++) {
                    color_raw_Hz[i] = 0.0f;
                    color_avg_Hz[i] = 0.0f;
                    color_cal[i] = 0.0f;
                }
                color_num = 0;
                color_string = nullptr;

                enable_motors = 0;
                cycle_counter = 0;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        const long long main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}

/**
 *
 * @param motor_M1 Motor 1
 * @param motor_M2 Motor 2
 * @param active_sensor_bar Sensor bar to be used
 * @param direction Driving Direction (1 = forward, -1 = backward)
 * @param angle Angle measured by the line follower
 * @param Cwheel2robot
 * @param wheel_vel_max Max velocity
 * @param r_wheel wheel radius
 */
void driveRobot(DCMotor &motor_M1,
                DCMotor &motor_M2,
                float &direction,
                float &angle,
                const Eigen::Matrix2f &Cwheel2robot,
                float wheel_vel_max,
                float r_wheel,
                float speed)
{
    constexpr float Kp{5.0f};

    // smoothing + softer response for small angles
    static float angle_filtered = 0.0f;
    const float alpha = 0.3f; // low-pass filter factor (smaller = smoother)
    angle_filtered = (1.0f - alpha) * angle_filtered + alpha * angle;

    float abs_angle = fabs(angle_filtered);

    // reduce gain for small angles to avoid oscillation
    float effective_Kp = (abs_angle < 0.1f) ? (Kp * 0.3f) : Kp;

    // smooth acceleration (ramp up speed)
    static float speed_filtered = 0.0f;

    if (speed_filtered_reset == 0.0f) {
        speed_filtered = 0.0f;
    }
    // faster convergence when close to target speed
    float diff = speed - speed_filtered;

    const float accel_alpha_slow = 0.1f; // smooth start
    const float accel_alpha_fast = 0.3f; // faster convergence

    float accel_alpha = (fabs(diff) < 0.1f) ? accel_alpha_fast : accel_alpha_slow;

    speed_filtered = (1.0f - accel_alpha) * speed_filtered + accel_alpha * speed;
    speed_filtered_reset = 1.0f;

    const Eigen::Vector2f robot_coord = {speed_filtered * wheel_vel_max * r_wheel, -effective_Kp * angle_filtered};
    const Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;

    motor_M1.setVelocity((wheel_speed(0) / (2.0f * M_PIf)) * direction);
    motor_M2.setVelocity(((wheel_speed(1) / (2.0f * M_PIf)) * -1) * direction);
}

void stopRobot(DCMotor &motor_M1, DCMotor &motor_M2)
{
    motor_M1.setVelocity(0.0f);
    motor_M2.setVelocity(0.0f);
    speed_filtered_reset = 0.0f;
}

/**
 *
 * @param sensor_bar
 * @return int (0 = normal line detected, 1 = small line detected, 2 = big line detected)
 */
int detectLine(SensorBar *&sensor_bar)
{
    float raw = sensor_bar->getRaw();
    // printf("raw: %f\n", raw);

    switch (static_cast<int>(raw)) {
        case 255:
            return 2;
        case 60:
            return 1;
        default:
            return 0;
    }
}

void measureColor(
    ColorSensor &sensor, float raw[4], float avg[4], float cal[4], int &color_num, const char *&color_string)
{
    for (int i = 0; i < 4; i++) {
        raw[i] = sensor.readRawColor()[i];
        avg[i] = sensor.readColor()[i];
        cal[i] = sensor.readColorCalib()[i];
    }

    color_num = sensor.getColor();
    color_string = sensor.getColorString(color_num);
}