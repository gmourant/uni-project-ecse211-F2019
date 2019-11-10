package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import java.util.Map;
import ca.mcgill.ecse211.project.UltrasonicPoller;
import ca.mcgill.ecse211.project.Display;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * @author Aakarsh
 * @author Steven
 * @author Hassan
 */

public class Main {
  //initilalizes Ultrasonic Sensor
  static SampleProvider distance = US_SENSOR.getMode("Distance");
  static float[] sampleUS = new float[distance.sampleSize()];
  final static UltrasonicPoller UP = new UltrasonicPoller(distance, sampleUS);
  
  static double tunnelStartX = 0;  // should be middle of tunnel entry
  static double tunnelStartY = 0;

  static double tunnelEndX = 0;
  static double tunnelEndY = 0;

  static double tunnelTheta = 0;

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
    
    Region tunnel = tnr;

    if (buttonChoice == Button.ID_LEFT) {
      // RED TEAM
      tunnel = tnr;

    }
    else if (buttonChoice == Button.ID_RIGHT) {
      // GREEN TEAM
      tunnel = tng;
    }
    
    // start odometer thread
    // start display thread
    new Thread(odometer).start();
    new Thread(new Display()).start();

    // localize
    localize();
    
    // compute tunnel coordinates
    computeTunnelCoordinates(tunnel);

    // navigate to tunnel entrance
    // turn to face tunnel
    // navigate through tunnel
    Navigation.travelTo(tunnelStartX, tunnelStartY);
    Navigation.turnTo(tunnelTheta);
    Navigation.travelTo(tunnelEndX, tunnelEndY);

    // TODO: ensure robot stays within island
    // calculate and move to launch point
    Navigation.launchPosition(bin.x, bin.y, RADIUS);
    Navigation.travelTo(bin.x, bin.y, RADIUS);

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

    localize = new UltrasonicLocalizer(1, US_SENSOR);
    localize.localize();

    //Start light localization when ultrasonic localizationi is over
    lightLocalize.localize();
  }
  
  /**
   * Method to compute the coordinates of the tunnel that the robot moves to
   */
  public static void computeTunnelCoordinates(Region tunnel) {
    double x0;
    double y0;
    
    double x1;
    double y1;
    
    if(horizontalTunnel(tunnel)) {
      x0 = tunnel.ll.x - TILE_SIZE/2;     // so that we dont crash into tunnel walls
      y0 = tunnel.ll.y + TILE_SIZE/2;

      x1 = tunnel.ur.x + TILE_SIZE/2;
      y1 = tunnel.ur.y - TILE_SIZE/2;

      tunnelTheta = 90;     // pointing right
    }
    else {      // vertical tunnel
      x0 = tunnel.ll.x + TILE_SIZE/2;     // so that we dont crash into tunnel walls
      y0 = tunnel.ll.y - TILE_SIZE/2;

      x1 = tunnel.ur.x - TILE_SIZE/2;
      y1 = tunnel.ur.y + TILE_SIZE/2;

      tunnelTheta = 0;     // pointing up
    }
    
    double pos[] = odometer.getXYT();
    double robotX = pos[0];
    double robotY = pos[1];
    
    // which of the computed points tunnel starts at (closest)
    double dist1 = Math.sqrt(Math.pow(robotX-x0, 2) + Math.pow(robotY-y0, 2));
    double dist2 = Math.sqrt(Math.pow(robotX-x1, 2) + Math.pow(robotY-y1, 2));
    
    if(dist1 <= dist2) {
      tunnelStartX = x0;
      tunnelStartY = y0;
      tunnelEndX = x1;
      tunnelEndY = y1;
    }
    else {
      tunnelStartX = x1;
      tunnelStartY = y1;
      tunnelEndX = x0;
      tunnelEndY = y0;
      
      tunnelTheta -= 180;
    }
  }
  
  /**
   * computes the orientation of the tunnel
   * 
   * @param tunnel  region
   * @return true if horizontal orientation of tunnel, false if vertical
   */
  public static boolean horizontalTunnel(Region tunnel) {
    return (Math.abs(tunnel.ur.y - tunnel.ll.y)==1);
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
