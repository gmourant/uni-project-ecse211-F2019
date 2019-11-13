package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;

/**
 * Abstraction of the Launcher mounted on the robot. When the launch method is called, it starts the launch loop, which
 * consists of waiting for 5 seconds, setting the acceleration of the launcher motor to 100 degrees/sec/sec and rotating
 * it 360 degrees, and finally stopping the motor.
 * 
 * @author Steven
 * @author Aakarsh
 * @version 1.2.2
 * @since 1.1.1
 */
public class Launcher {
  // setup launching mechanism
  public static void launch() {
    leftMotor.stop(true);
    rightMotor.stop(false);
    for (int i = 0; i < 2; i++) {
      Main.sleepFor(2500);

      // launch
      launchMotor.setAcceleration(100);
      //ratio of driver gear teeth and driven gear is 5:9 = 360 degree:648 degree
      launchMotor.rotate(648);

      // stop for reloading
      launchMotor.stop();
    }
  }
}

