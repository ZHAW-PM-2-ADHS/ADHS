#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "Servo.h"
 // test comment
 
bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end

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




    

    //servo variables
    float servo_input = 0.0f;
    //float step = 0.01f; //step size for servo 

    //variable for servo base
    float startPos = 0.5f; //starting position 
    float firstSlotPos = 0.7f; //first slot position
    float secondSlotPos = 0.8f; //second slot position
    float thirdSlotPos = 0.9f; //third slot position
    float endPos = 1.0f; //end position 

    //variables for servo arm
    float retractedPos = 0.7f; // retracted position
    float lowPackagePos = 0.98f; // low package position
    float highPackagePos = 0.9f; // high package position
    float storePos = 0.85f; // position to store the package in the arm

    //variables for servo mechanism
    float openPos = 0.55f; // start position
    float closedPos = 1.0f; // end position



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

    // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
    servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max); 
    servo_D1.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);
    servo_D2.calibratePulseMinMax(servo_D2_ang_min, servo_D2_ang_max);

    servo_D0.setMaxAcceleration(0.5f);
    servo_D1.setMaxAcceleration(0.5f);
    servo_D2.setMaxAcceleration(0.5f);
    

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---
        
        // print to serial monitor
        printf("Pulse Width: %f \n", servo_input);

        if (do_execute_main_task) {

            // --- code that runs when the user or blue button was pressed goes here ---
            
            // enable servo
            if (!servo_D0.isEnabled())
                servo_D0.enable(startPos);
            if (!servo_D1.isEnabled())
                servo_D1.enable(retractedPos);
            if (!servo_D2.isEnabled())
                servo_D2.enable(closedPos);

            
            
        //picking up the package 
        thread_sleep_for(2000); // wait for 5 seconds
        servo_D1.setPulseWidth(lowPackagePos);
             

        servo_D2.setPulseWidth(openPos);   
            thread_sleep_for(800); // wait for 5 seconds

                servo_D2.setPulseWidth(closedPos);   
                    thread_sleep_for(1500); // wait for 5 seconds
                
                    servo_D1.setPulseWidth(retractedPos);
                        thread_sleep_for(1000); // wait for 5 seconds

        //move to to first slot 
        servo_D0.setPulseWidth(firstSlotPos);
                thread_sleep_for(2000); // wait for 5 seconds
                    
                    servo_D1.setPulseWidth(storePos);
                        thread_sleep_for(2000); // wait for 5 seconds

                            servo_D2.setPulseWidth(openPos);   
                                thread_sleep_for(1000); // wait for 5 seconds

                                    servo_D2.setPulseWidth(closedPos);   
                                        thread_sleep_for(1000); // wait for 5 seconds

                                            servo_D1.setPulseWidth(retractedPos);
                                                thread_sleep_for(2000); // wait for 5 seconds

        //get second package 
        servo_D0.setPulseWidth(startPos);
                thread_sleep_for(2000); // wait for 5 seconds
                    
                    servo_D1.setPulseWidth(lowPackagePos);

                            servo_D2.setPulseWidth(openPos);   
                                thread_sleep_for(800); // wait for 5 seconds

                                    servo_D2.setPulseWidth(closedPos);   
                                        thread_sleep_for(1500); // wait for 5 seconds

                                            servo_D1.setPulseWidth(retractedPos);
                                                thread_sleep_for(1000); // wait for 5 seconds


        //move to to second slot 
        servo_D0.setPulseWidth(secondSlotPos);
                thread_sleep_for(2000); // wait for 5 seconds
                    
                    servo_D1.setPulseWidth(storePos);
                        thread_sleep_for(2000); // wait for 5 seconds

                            servo_D2.setPulseWidth(openPos);   
                                thread_sleep_for(1000); // wait for 5 seconds

                                    servo_D2.setPulseWidth(closedPos);   
                                        thread_sleep_for(1000); // wait for 5 seconds

                                            servo_D1.setPulseWidth(retractedPos);
                                                thread_sleep_for(2000); // wait for 5 seconds

        
        //get third package 
        servo_D0.setPulseWidth(startPos);
                thread_sleep_for(2000); // wait for 5 seconds
                    
                    servo_D1.setPulseWidth(lowPackagePos);

                            servo_D2.setPulseWidth(openPos);   
                                thread_sleep_for(800); // wait for 5 seconds

                                    servo_D2.setPulseWidth(closedPos);   
                                        thread_sleep_for(1500); // wait for 5 seconds

                                            servo_D1.setPulseWidth(retractedPos);
                                                thread_sleep_for(1000); // wait for 5 seconds


        //move to to third slot 
        servo_D0.setPulseWidth(thirdSlotPos);
                thread_sleep_for(2000); // wait for 5 seconds
                    
                    servo_D1.setPulseWidth(storePos);
                        thread_sleep_for(2000); // wait for 5 seconds

                            servo_D2.setPulseWidth(openPos);   
                                thread_sleep_for(1000); // wait for 5 seconds

                                    servo_D2.setPulseWidth(closedPos);   
                                        thread_sleep_for(1000); // wait for 5 seconds

                                            servo_D1.setPulseWidth(retractedPos);
                                                thread_sleep_for(2000); // wait for 5 seconds


        //get fourth package 
        servo_D0.setPulseWidth(startPos);
                thread_sleep_for(2000); // wait for 5 seconds
                    
                    servo_D1.setPulseWidth(lowPackagePos);

                            servo_D2.setPulseWidth(openPos);   
                                thread_sleep_for(800); // wait for 5 seconds

                                    servo_D2.setPulseWidth(closedPos);   
                                        thread_sleep_for(1500); // wait for 5 seconds

                                            servo_D1.setPulseWidth(retractedPos);
                                                thread_sleep_for(1000); // wait for 5 seconds



        //move to to fourth slot 
        servo_D0.setPulseWidth(endPos);
                thread_sleep_for(2000); // wait for 5 seconds
                    
                    servo_D1.setPulseWidth(storePos);
                        thread_sleep_for(2000); // wait for 5 seconds

                            servo_D2.setPulseWidth(openPos);   
                                thread_sleep_for(1000); // wait for 5 seconds

                                    servo_D2.setPulseWidth(closedPos);   
                                        thread_sleep_for(1000); // wait for 5 seconds

                                            servo_D1.setPulseWidth(retractedPos);
                                                thread_sleep_for(2000); // wait for 5 seconds

        servo_D0.setPulseWidth(startPos);
                thread_sleep_for(5000); // wait for 5 seconds  
            
            
            

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                
                led1 = 0;
                servo_D0.disable();
                servo_D1.disable();
                servo_D2.disable();
                
                
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
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
