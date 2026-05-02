#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include <Eigen/Dense>

#include "ColorSensor.h"
#include "DCMotor.h"
#include "DebounceIn.h"
#include "SensorBar.h"
#include "Servo.h"

#define M_PIf 3.14159265358979323846f // pi

// variable for servo base
constexpr float D0_startPos = 0.5f;      // starting position
constexpr float D0_redSlotPos = 0.7f;    // red slot position
constexpr float D0_greenSlotPos = 0.8f;  // green slot position
constexpr float D0_blueSlotPos = 0.9f;   // blue slot position
constexpr float D0_yellowSlotPos = 1.0f; // yellow slot position

// variables for servo arm
float D1_retractedPos = 0.7f;   // retracted position
float D1_lowPackagePos = 0.98f; // low package position
float D1_highPackagePos = 0.9f; // high package position
float D1_storePos = 0.85f;      // position to store the package in the arm

// variables for servo mechanism
float D2_openPos = 0.55f;  // start position
float D2_closedPos = 1.0f; // end position

// minimal pulse width and maximal pulse width obtained from the servo calibration process
// reely S0090
float servo_D0_ang_min = 0.032f; // careful, these values might differ from servo to servo
float servo_D0_ang_max = 0.119f;
// reely S0090
float servo_D1_ang_min = 0.032f;
float servo_D1_ang_max = 0.119f;
// reely S0008
float servo_D2_ang_min = 0.025f;
float servo_D2_ang_max = 0.119f;

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

/**
 * @brief Drive the robot a certain distance at a constant speed.
 *
 * @param motor_M1 Reference to motor M1
 * @param motor_M2 Reference to motor M2
 * @param r_wheel Wheel radius in meters
 * @param distance Distance to drive in meters
 * @param speed Normalized speed (0.0 to 1.0)
 * @param main_task_period_ms Task period in milliseconds
 * @return true if distance has been reached, false otherwise
 */
bool driveDistance(
    DCMotor &motor_M1, DCMotor &motor_M2, float r_wheel, float distance, float speed, int main_task_period_ms);

void stopRobot(DCMotor &motor_M1, DCMotor &motor_M2);

int detectLine(SensorBar *&sensor_bar);

void measureColor(
    ColorSensor &sensor, float raw[4], float avg[4], float cal[4], int &color_num, const char *&color_string);

float colorNumToPosition(int color_num);

bool move_servo();

bool pickup_package(Servo &servo_D0, Servo &servo_D1, Servo &servo_D2, int color_num);

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

    // servo
    Servo servo_D0(PB_D0);
    Servo servo_D1(PB_D1);
    Servo servo_D2(PB_D2);

    // servo variables
    float servo_input = 0.0f;
    // float step = 0.01f; //step size for servo

    servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
    servo_D1.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);
    servo_D2.calibratePulseMinMax(servo_D2_ang_min, servo_D2_ang_max);

    // default acceleration of the servo motion profile is 1.0e6f
    servo_D0.setMaxAcceleration(0.7f);
    servo_D1.setMaxAcceleration(0.7f);
    servo_D2.setMaxAcceleration(0.7f);

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

    float speed = 0.5f; // translational speed scaling factor (0.0 to 1.0)

    // motor M1 and M2, do NOT enable motion planner when used with the LineFollower (disabled per default)
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    // transforms wheel to robot velocities
    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot << r_wheel / 2.0f, r_wheel / 2.0f, r_wheel / b_wheel, -r_wheel / b_wheel;

    float sensor_direction = -1.0f;

    // Both sensor bars, one in front, one at the back
    SensorBar sensor_bar_1(PB_9, PB_8, bar_dist);  // PB_9=D14, PB_8=D15
    SensorBar sensor_bar_2(PB_3, PB_10, bar_dist); // PB_3=D3, PB_10=D6

    // Pointer to the sensor bar actually being used
    SensorBar *sensor_bar_front = &sensor_bar_1;
    SensorBar *sensor_bar_back = &sensor_bar_2;

    // angle measured from sensor bar (black line) relative to robot
    float angle{0.0f};

    // rotational velocity controller max wheel velocity [rad/s]
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
    constexpr float starting_angle = -0.2;
    constexpr int starting_cycle_count = 40;

    // Specific line follower readings (at the back) that help avoiding a stop at the crossing to the warehouse
    constexpr int backSensorReadingsToNotStop[] = {1, 3, 7, 12, 14, 15, 48, 96, 112, 128, 192, 224, 240};

    // this loop will run forever
    while (true) {

        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {

            // Moving the Servo to a position on the top
            if (just_started) {
                if (!servo_D0.isEnabled())
                    servo_D0.enable(D0_startPos);
                if (!servo_D1.isEnabled())
                    servo_D1.enable(D1_retractedPos);
                if (!servo_D2.isEnabled())
                    servo_D2.enable(D2_openPos);
            }

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

                    measureColor(Color_Sensor, color_raw_Hz, color_avg_Hz, color_cal, color_num, color_string);

                    // If the detected color is not in the valid range, it is necessary to wait until a good reading comes
                    if (color_num >= 3 && color_num <= 8) {

                        // Only when pickup_package returns true (=pickup done), can the robot resume driving
                        if (pickup_package(servo_D0, servo_D1, servo_D2, color_num)) {
                            robot_stopped = false;
                            skipping_line_detection = true;
                        }
                    }
                }
            }

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

                servo_D0.disable();
                servo_D1.disable();
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

bool move_servo(Servo &servo, const float pulseWidth)
{
    servo.setPulseWidth(pulseWidth);
    return servo.isAtTarget();
}

bool pickup_package(Servo &servo_D0, Servo &servo_D1, Servo &servo_D2, const int color_num)
{
    enum State {
        INIT,
        LOWER_ARM,
        CLOSE_GRIPPER,
        LIFT_PACKAGE,
        ROTATE_TO_SLOT,
        STORE_PACKAGE,
        OPEN_GRIPPER,
        RESET_POSITION
    };

    // This variable persists across cycles
    static State state = INIT;

    switch (state) {
        case INIT:
            if (move_servo(servo_D0, D0_startPos) && move_servo(servo_D2, D2_openPos)) {
                state = LOWER_ARM;
            }
            break;
        case LOWER_ARM:
            if (move_servo(servo_D1, D1_lowPackagePos)) {
                state = CLOSE_GRIPPER;
            }
            break;
        case CLOSE_GRIPPER:
            if (move_servo(servo_D2, D2_closedPos)) {
                state = LIFT_PACKAGE;
            }
            break;
        case LIFT_PACKAGE:
            if (move_servo(servo_D1, D1_retractedPos)) {
                state = ROTATE_TO_SLOT;
            }
            break;
        case ROTATE_TO_SLOT:
            if (move_servo(servo_D0, colorNumToPosition(color_num))) {
                state = STORE_PACKAGE;
            }
            break;
        case STORE_PACKAGE:
            if (move_servo(servo_D1, D1_storePos)) {
                state = OPEN_GRIPPER;
            }
            break;
        case OPEN_GRIPPER:
            if (move_servo(servo_D2, D2_openPos)) {
                state = RESET_POSITION;
            }
            break;
        case RESET_POSITION:
            servo_D0.setPulseWidth(D0_startPos);
            servo_D1.setPulseWidth(D1_retractedPos);
            servo_D2.setPulseWidth(D2_openPos);
            state = INIT;
            return true;
    }
    return false;
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
    // Proportional gain for steering [1/s]
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

bool driveDistance(
    DCMotor &motor_M1, DCMotor &motor_M2, float r_wheel, float distance, float speed, int main_task_period_ms)
{
    // max physical velocity in rotations per second
    float max_physical_vel_rps = motor_M1.getMaxPhysicalVelocity();
    // actual target velocity in m/s
    float target_vel_ms = speed * max_physical_vel_rps * (2.0f * M_PIf) * r_wheel;
    // target velocity in rps
    float target_vel_rps = speed * max_physical_vel_rps;

    // total time to drive the distance
    float total_time = distance / target_vel_ms;
    // total number of cycles
    int total_cycles = static_cast<int>(total_time / (static_cast<float>(main_task_period_ms) * 0.001f));

    static int cycle_count = 0;

    if (cycle_count < total_cycles) {
        motor_M1.setVelocity(target_vel_rps);
        motor_M2.setVelocity(-target_vel_rps);
        cycle_count++;
        return false;
    } else {
        stopRobot(motor_M1, motor_M2);
        cycle_count = 0;
        return true;
    }
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

float colorNumToPosition(int color_num)
{
    switch (color_num) {
        case 3: // RED
            return D0_redSlotPos;
        case 5: // GREEN
            return D0_greenSlotPos;
        case 7: // BLUE
            return D0_blueSlotPos;
        case 4: // YELLOW
            return D0_yellowSlotPos;
        default:
            printf("Invalid color_num: %d\n", color_num);
            return D0_startPos;
    }
}