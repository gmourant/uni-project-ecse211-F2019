package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.project.UltrasonicPoller;
import ca.mcgill.ecse211.project.Display;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;

/**
 * 
 * @author Aakarsh
 * @author Steven
 * @author Hassan
 */

public class Main {
  //initilalizes Ultrasonic Sensor
  static SampleProvider distance = US_SENSOR.getMode("Distance");
  static float[] sampleUS = new float[distance.sampleSize()];
  final static UltrasonicPoller UP = new UltrasonicPoller(distance, sampleUS);

  public static void main(String[] args) {
    LightLocalizer lightLocalize = new LightLocalizer();
    UltrasonicLocalizer localize;
    new Thread(new Display()).start();
    new Thread(odometer).start();
    
    // display the launch and final positions
    double[] launchPos = Navigation.getLaunchPosition(TARGET_POSITION[0], TARGET_POSITION[1], RADIUS);
    
    // wait for center button press
    int buttonChoice;
    do {
      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_ENTER);
    
    localize = new UltrasonicLocalizer(1, US_SENSOR);
    localize.localize();
    //Start light localization when ultrasonic localization is over
    lightLocalize.localize();
    
    //position yourself before shooting
    navigate.travelTo(TARGET_POSITION[0], TARGET_POSITION[1], RADIUS);
    
    //shoot the ping pong balls
    Launcher.launch();
  }

  /**
   * Sleeps current thread for the specified duration.
   * 
   * @param duration sleep duration in milliseconds
   */
  public static void sleepFor(long duration) {
    try {
      Thread.sleep(duration);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
  }

  /**
   * Shows error and exits program.
   * 
   * @param String
   */
  public static void showErrorAndExit(String errorMessage) {
    LCD.clear();
    System.err.println(errorMessage);

    // Sleep for 2 seconds so user can read error message
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
    }

    System.exit(-1);
  }
}
