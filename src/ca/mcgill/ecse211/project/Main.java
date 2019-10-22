package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.project.UltrasonicPoller;
import ca.mcgill.ecse211.project.Display;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * 
 * @author Aakarsh
 * @author Steven
 * @author Hassan
 */

public class Main {
  // initilalizes Ultrasonic Sensor
  static SampleProvider distance = US_SENSOR.getMode("Distance");
  static float[] sampleUS = new float[distance.sampleSize()];
  final static UltrasonicPoller UP = new UltrasonicPoller(distance, sampleUS);

  public static void main(String[] args) {

    int buttonChoice;

    do {
      LCD.clear();

      // notify user when to start
      LCD.drawString("< Left    | Right >", 0, 0);
      LCD.drawString("          |        ", 0, 1);
      LCD.drawString("Stationary| Mobile ", 0, 2);
      LCD.drawString("Launch    | Launch ", 0, 3);
      LCD.drawString("          |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    LCD.clear();

    if (buttonChoice == Button.ID_LEFT) {
      Launcher.launch();
    } else if (buttonChoice == Button.ID_RIGHT) {
      LightLocalizer lightLocalize = new LightLocalizer();
      UltrasonicLocalizer localize = new UltrasonicLocalizer(0, US_SENSOR);
      new Thread(new Display()).start();
      new Thread(odometer).start();
      
      do {
        buttonChoice = Button.waitForAnyPress();
      } while (buttonChoice != Button.ID_ENTER);
      
      // wait for center button press
      localize.localize();
      // Start light localization when ultrasonic localization is over
      lightLocalize.localize();

      Sound.twoBeeps();

      // position yourself before shooting
      navigate.travelTo(TARGET_POSITION[0],TARGET_POSITION[1],RADIUS);
      Sound.twoBeeps();

      // shoot the ping pong balls
      Launcher.launch();
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE); // do nothing

    System.exit(0);
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
