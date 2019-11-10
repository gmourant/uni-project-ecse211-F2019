package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;

/**
 * Abstraction of the Launcher mounted on the robot.
 * When the launch method is called, it starts the launch loop,
 * which consists of waiting for 5 seconds, setting the acceleration of
 * the launcher motor to 100 degrees/sec/sec and rotating it 360 degrees,
 * and finally stopping the motor.
 * 
 * @author Steven
 * @author Aakarsh
 * @version 1.2.1
 * @since 1.1.1
 */
public class Launcher {
  // setup launching mechanism
  public static void launch() {
    while(true) {
      Main.sleepFor(5000);
      
      //launch
      launchMotor.setAcceleration(100);
      launchMotor.rotate(360);
      
      //stop for reloading
      launchMotor.stop();
    }
  }
}

