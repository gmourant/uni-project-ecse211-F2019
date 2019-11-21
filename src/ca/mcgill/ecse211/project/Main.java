package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import java.util.Map;
import ca.mcgill.ecse211.project.UltrasonicPoller;
import ca.mcgill.ecse211.project.Display;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * Starting point of the code. The Main class performs the necessary calls to complete the tasks required in the demo.
 * The Main class sets which region the robot is in based on user input. The Region defines which tunnel the robot will
 * need to use to move to the next island. It then initializes the Odometer and Display threads, and calls the localize
 * function, which consists Light Localization and Ultrasonic Localization. After that, Main performs the required
 * navigation to and through the tunnel using methods in the Navigation class. Finally, Main navigates to computed
 * launch point and calls the launch method in Launcher class.
 * 
 * @author Aakarsh
 * @author Steven
 * @author Hassan
 * @version 1.2.1
 * @since 1.1.1
 */
public class Main {
  // initilalizes Ultrasonic Sensor
  static SampleProvider distance = US_SENSOR.getMode("Distance");
  static float[] sampleUS = new float[distance.sampleSize()];
  final static UltrasonicPoller UP = new UltrasonicPoller(distance, sampleUS);

  public static void main(String[] args) {
    new Thread(odometer).start();
    new Thread(new Display()).start(); // TODO Comment out when presenting

    while (Resources.notGotWifi == true) {
      Resources.wifiParameters = null;
      receiveWifiParameters();
    }
    System.out.println("Map:\n" + wifiParameters);
    LCD.clear();
    new Thread(new Display()).start(); // TODO Comment out when presenting
    Region tunnel = tng;
    Region startIsland = green;
    Point bin = greenBin;
    int startingCorner = greenCorner;

    if (redTeam == 2) {
      // RED TEAM
      tunnel = tnr;
      startIsland = red;
      bin = redBin;
      startingCorner = redCorner;

    } else if (greenTeam == 2) {
      // GREEN TEAM
      tunnel = tng;
      startIsland = green;
      bin = greenBin;
      startingCorner = greenCorner;
    }

    // localize
    localize(startingCorner);

    Sound.beep();
    Sound.beep();
    Sound.beep();

    // compute tunnel coordinates
    computeTunnelCoordinates(tunnel);
    // Navigation.travelTo(2, 2);

    // navigate to tunnel entrance
    // turn to face tunnel
    // navigate through tunnel
    Navigation.travelTo(tunnelStartX, tunnelStartY);

    Navigation.turnTo(tunnelTheta);
    localizeForward(tunnelTheta, true);

    Navigation.travelTo(tunnelEndX, tunnelEndY);

    /*
     * 4.Each machine localizes to the grid. When completed, the machine must stop and issue a sequence of 3 beeps.
     * 
     */

    Sound.beep();
    Sound.beep();
    Sound.beep();

    localizeForward(tunnelTheta, false);


    // TODO: ensure robot stays within island
    // calculate and move to launch point
    Navigation.travelTo(bin.x, bin.y, RADIUS);

    /*
     * 5.Each machine navigates to their corresponding tunnel, transits, and then proceeds to their launch point. Upon
     * arriving, each machine will again stop and issue a sequence of 3 beeps.
     */

    Sound.beep();
    Sound.beep();
    Sound.beep();

    // launch balls
    Launcher.launch();
    computeTunnelCoordinates(tunnel);
    Navigation.travelTo(tunnelStartX, tunnelStartY);
    Navigation.turnTo(tunnelTheta);
    localizeForward(tunnelTheta, true);
    Navigation.travelTo(startIsland.ll.x, startIsland.ll.y);

    /*
     * 8.Upon returning to the starting corner, each robot halts and issues a sequence of 5 beeps.
     */
    Sound.beep();
    Sound.beep();
    Sound.beep();
    Sound.beep();
    Sound.beep();


    while (Button.waitForAnyPress() != Button.ID_ESCAPE); // do nothing

    System.exit(0);
  }

  /**
   * Method to run the robot localization First, runs ultrasonic localization, then runs the light localization and
   * corrects theta
   */
  public static void localize(int startingCorner) {
    LightLocalizer lightLocalize = new LightLocalizer();
    UltrasonicLocalizer localize;

    localize = new UltrasonicLocalizer(0, US_SENSOR);
    localize.localize();

    // Start light localization when ultrasonic localizationi is over
    lightLocalize.localize();
    if (startingCorner == 1) {
      // GREEN TEAM
      odometer.setXYT(TILE_SIZE * 14, TILE_SIZE, 0);
    } else if (startingCorner == 2) {
      // RED TEAM
      odometer.setXYT(TILE_SIZE * 14, TILE_SIZE * 8, 270);
    } else if (startingCorner == 3) {
      // RED TEAM
      odometer.setXYT(TILE_SIZE, TILE_SIZE * 8, 180);
    }
  }

  /**
   * Aligns device with line as it moves forward also corrects its angle
   * 
   * @param angle
   */
  public static void localizeForward(double angle, boolean backup) {
    LightLocalizer lightLocalize = new LightLocalizer();
    lightLocalize.localizeForward(angle, backup);
  }

  /**
   * Method to compute the coordinates of the tunnel that the robot moves to
   */
  public static void computeTunnelCoordinates(Region tunnel) {
    double x0;
    double y0;

    double x1;
    double y1;

    if (horizontalTunnel(tunnel)) {
      x0 = tunnel.ll.x - 0.5; // so that we dont crash into tunnel walls
      y0 = tunnel.ll.y + 0.5;

      x1 = tunnel.ur.x + 0.5;
      y1 = tunnel.ur.y - 0.5;

      tunnelTheta = 90; // pointing right
    } else { // vertical tunnel
      x0 = tunnel.ll.x + 0.5; // so that we dont crash into tunnel walls
      y0 = tunnel.ll.y - 0.5;

      x1 = tunnel.ur.x - 0.5;
      y1 = tunnel.ur.y + 0.5;

      tunnelTheta = 0; // pointing up
    }

    double pos[] = odometer.getXYT();
    double robotX = pos[0] / TILE_SIZE;
    double robotY = pos[1] / TILE_SIZE;

    // which of the computed points tunnel starts at (closest)
    double dist1 = Math.sqrt(Math.pow(robotX - x0, 2) + Math.pow(robotY - y0, 2));
    double dist2 = Math.sqrt(Math.pow(robotX - x1, 2) + Math.pow(robotY - y1, 2));

    if (dist1 <= dist2) {
      tunnelStartX = x0;
      tunnelStartY = y0;
      tunnelEndX = x1;
      tunnelEndY = y1;
    } else {
      tunnelStartX = x1;
      tunnelStartY = y1;
      tunnelEndX = x0;
      tunnelEndY = y0;

      tunnelTheta -= 180;
    }
    if (horizontalTunnel(tunnel)) {
      tunnelStartX -= 0.5;
    } else
      tunnelStartY -= 0.5;
  }

  /**
   * computes the orientation of the tunnel
   * 
   * @param tunnel region
   * @return true if horizontal orientation of tunnel, false if vertical
   */
  public static boolean horizontalTunnel(Region tunnel) {
    return (Math.abs(tunnel.ur.y - tunnel.ll.y) == 1);
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
