package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import java.util.Map;
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
  //initilalizes Ultrasonic Sensor
  static SampleProvider distance = US_SENSOR.getMode("Distance");
  static float[] sampleUS = new float[distance.sampleSize()];
  final static UltrasonicPoller UP = new UltrasonicPoller(distance, sampleUS);

  public static void main(String[] args) {

    System.out.println("Map:\n" + wifiParameters);

    int buttonChoice;
    do {
      LCD.clear();

      // notify user when to start
      LCD.drawString("< Left    | Right >", 0, 0);
      LCD.drawString("          |        ", 0, 1);
      LCD.drawString(" Red      |  Green ", 0, 2);
      LCD.drawString(" Team     |   Team ", 0, 3);
      LCD.drawString("          |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    LCD.clear();

    double tunnelStartX = 0;  // should be middle of tunnel entry
    double tunnelStartY = 0;

    double tunnelEndX = 0;
    double tunnelEndY = 0;

    double tunnelTheta = 0;

    if (buttonChoice == Button.ID_LEFT) {
      // RED TEAM
      tunnelStartX = tnr.ll.x - TILE_SIZE/2;     // so that we dont crash into tunnel walls
      tunnelStartY = tnr.ur.y - TILE_SIZE/2;

      tunnelEndX = tnr.ur.x;
      tunnelEndY = tnr.ur.y - TILE_SIZE/2;

      tunnelTheta = 90;     // pointing right

    }
    else if (buttonChoice == Button.ID_RIGHT) {
      // GREEN TEAM
      tunnelStartX = tng.ll.x + TILE_SIZE/2;     // so that we dont crash into tunnel walls
      tunnelStartY = tng.ur.y - TILE_SIZE/2;

      tunnelEndX = tng.ur.x - TILE_SIZE/2;
      tunnelEndY = tng.ur.y;

      tunnelTheta = 0;     // pointing up
    }

    // start odometer thread
    // start display thread
    new Thread(odometer).start();
    new Thread(new Display()).start();

    // localize
    localize();

    // navigate to tunnel entrance
    // turn to face tunnel
    // navigate through tunnel
    Navigation.travelTo(tunnelStartX, tunnelStartY);
    Navigation.turnTo(tunnelTheta);
    Navigation.travelTo(tunnelEndX, tunnelEndY);

    // TODO: ensure robot stays within island
    // calculate and move to launch point
    Navigation.launchPosition(bin.x, bin.y, RADIUS);

    // launch balls
    Launcher.launch();

    while (Button.waitForAnyPress() != Button.ID_ESCAPE); // do nothing

    System.exit(0);
  }

  /**
   * Method to run the robot localization
   * First, runs ultrasonic localization, then runs the light localization and corrects theta
   */
  public static void localize() {
    LightLocalizer lightLocalize = new LightLocalizer();
    UltrasonicLocalizer localize;

    new Thread(new Display()).start();
    new Thread(odometer).start();

    localize = new UltrasonicLocalizer(1, US_SENSOR);
    localize.localize();

    //Start light localization when ultrasonic localizationi is over
    lightLocalize.localize();
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
