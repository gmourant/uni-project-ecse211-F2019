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
    // new Thread(new Display()).start(); // TODO Comment out when presenting
    Region tunnel = tng;
    Region startIsland = green;
    Point bin = greenBin;
    int startingCorner = greenCorner;

    if (redTeam == TEAM_NUMBER) {
      // RED TEAM
      tunnel = tnr;
      startIsland = red;
      bin = redBin;
      startingCorner = redCorner;

    } else if (greenTeam == TEAM_NUMBER) {
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


    //
    // // compute tunnel coordinates
    computeTunnelCoordinates(tunnel);

    // navigate to tunnel entrance
    // turn to face tunnel
    // navigate through tunnel
    Navigation.travelTo(tunnelStartX, tunnelStartY);

    Navigation.turnTo(Math.toRadians(tunnelTheta));
    localizeForward(tunnelTheta, true, null);
    if (!checkOutOfBounds(tunnelTheta + 90, startIsland)) {
      Navigation.turnTo(Math.toRadians(tunnelTheta + 90));
      localizeForward(tunnelTheta + 90, true, null);
      Navigation.goMid(true);
    } else {
      Navigation.turnTo(Math.toRadians(tunnelTheta - 90));
      localizeForward(tunnelTheta - 90, true, null); 
      Navigation.goMid(true);
    }
    // // Navigation.turnTo(Math.toRadians(tunnelTheta - 90));
    // // localizeForward(tunnelTheta - 90, true, null);
    // // Navigation.goMid();
    Navigation.turnTo(Math.toRadians(tunnelTheta));
    odometer.setXYT(tunnelStartX * TILE_SIZE, tunnelStartY * TILE_SIZE, tunnelTheta);
    Navigation.travelTo(tunnelEndX, tunnelEndY);

    /*
     * 4.Each machine localizes to the grid. When completed, the machine must stop and issue a sequence of 3 beeps.
     * 
     */
    //
    localizeBackward(tunnelTheta, true, island);
    //
    //
    // // TODO: ensure robot stays within island
    // // calculate and move to launch point
    // //Navigation.travelTo(bin.x, bin.y, RADIUS);

    /*
     * 5.Each machine navigates to their corresponding tunnel, transits, and then proceeds to their launch point. Upon
     * arriving, each machine will again stop and issue a sequence of 3 beeps.
     */

    // // Navigation.launchPosition(bin.x, bin.y, RADIUS);
    // // Navigation.travelTo(bin.x, bin.y, RADIUS);
    // // Navigation.travelTo(bin.x, bin.y);
    // // Navigation.turnTo(Math.toRadians(tnr.ur.x));
    navigateToLaunchPosition(bin);

    Sound.beep();
    Sound.beep();
    Sound.beep();

    // launch balls
    Launcher.launch();
    computeTunnelCoordinates(tunnel);
    if (ObstacleAvoidance.i != 0) {
      for (int j = ObstacleAvoidance.i; j >= 0; j--) {
        Navigation.travelTo(ObstacleAvoidance.validPoints[j].x / TILE_SIZE,
            ObstacleAvoidance.validPoints[j].y / TILE_SIZE);
      }
    }
    
    Navigation.travelTo(tunnelStartX, tunnelStartY);
    Navigation.turnTo(Math.toRadians(tunnelTheta));
    localizeForward(tunnelTheta, true, island);
    if (!checkOutOfBounds(tunnelTheta + 90, island)) {
      Navigation.turnTo(Math.toRadians(tunnelTheta + 90));
      localizeForward(tunnelTheta + 90, true, null);
      Navigation.goMid(true);
    } else {
      Navigation.turnTo(Math.toRadians(tunnelTheta - 90));
      localizeForward(tunnelTheta - 90, true, null);
      Navigation.goMid(true);
    }
    Navigation.turnTo(Math.toRadians(tunnelTheta));
    if(horizontalTunnel(tunnel))
    odometer.setXYT((tunnelStartX+1) * TILE_SIZE, tunnelStartY * TILE_SIZE, tunnelTheta);
    else
      odometer.setXYT(tunnelStartX * TILE_SIZE, (tunnelStartY+1) * TILE_SIZE, tunnelTheta);

    Navigation.travelTo(tunnelEndX, tunnelEndY);
    navigateToStart(startingCorner);

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
   * 
   * @param startingCorner, 0,1,2 or 3 depending of parameters sent by wifi
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
  public static void localizeForward(double angle, boolean backup, Region region) {
    LightLocalizer lightLocalize = new LightLocalizer();
    lightLocalize.localizeForward(angle, backup);
  }
  
  public static void localizeBackward(double angle, boolean backup, Region region) {
    LightLocalizer lightLocalize = new LightLocalizer();
    lightLocalize.localizeBackward(angle, backup);
  }

  /**
   * Calculaes closest launch position to device and navigates to it
   * 
   * @param bin, Point of bin
   */
  public static void navigateToLaunchPosition(Point bin) {
    double[] pos = odometer.getXYT();
    double[] lp = Navigation.launchPosition(bin.x, bin.y, RADIUS, pos[0], pos[1]);
    if (!validPoint(lp[0], lp[1], island)) {
      lp = navigateToAlternateLaunchPosition(lp, bin);
    }

    Navigation.travelTo(lp[0] / TILE_SIZE, lp[1] / TILE_SIZE, bin, island);
    Navigation.turnTo(findAngle(odometer.getXYT()[0], odometer.getXYT()[1], bin.x, bin.y));
  }

  /**
   * Computes the angle between any 2 given points
   * 
   * @param curr_x current x position of the robot
   * @param curr_y current y position of the robot
   * @param x final x position
   * @param y final y position
   * @return angle between the 2 points
   */
  public static double findAngle(double curr_x, double curr_y, double x, double y) {
    return Math.atan2(x * TILE_SIZE - curr_x, y * TILE_SIZE - curr_y);
  }

  /**
   * In case the original launch position computed is not within the island, we compute alternative launch positions. We
   * do this by simply moving clockwise on the circle of radius RADIUS around the bin
   *
   * @param lp current launch position
   */
  public static double[] navigateToAlternateLaunchPosition(double[] lp, Point bin) {
    while (!validPoint(lp[0], lp[1], island.ll.x, island.ll.y, island.ur.x, island.ur.y)) {
      lp = Navigation.alternateLaunchPosition(bin, RADIUS, lp);
    }
    return lp;
  }

  /**
   * Computes whether given point (x,y) lies within the defined region
   *
   * @param x defines the given point
   * @param y defines the given point
   * @param ll_x lower left point for the defined region
   * @param ll_y lower left point for the defined region
   * @param ur_x upper right point for the defined region
   * @param ur_y upper right point for the defined region
   * @return true if point lies within the defined region, else false
   */
  public static boolean validPoint(double x, double y, double ll_x, double ll_y, double ur_x, double ur_y) {
    return (x > ll_x * TILE_SIZE && x < ur_x * TILE_SIZE && y > ll_y * TILE_SIZE && y < ur_y * TILE_SIZE);
  }

  /**
   * Computes whether given point (x,y) lies within the defined region
   * 
   * @param x defines the given point
   * @param y defines the given point
   * @param region
   * @return true if point lies within the defined region, else false
   */
  public static boolean validPoint(double x, double y, Region region) {
    return (x > region.ll.x * TILE_SIZE && x < region.ur.x * TILE_SIZE && y > region.ll.y * TILE_SIZE
        && y < region.ur.y * TILE_SIZE);
  }

  /**
   * Return true if place to localize is within region
   * 
   * @param angle, angle in degrees at which device desires to localize
   * @param region
   * @return true if place to localize is within region, else false
   */
  public static boolean checkOutOfBounds(double angle, Region region) {
    double x = odometer.getXYT()[0];
    double y = odometer.getXYT()[1];
    if (angle > 340 && angle < 20) {
      if (y + TILE_SIZE / 2 > region.ur.y * TILE_SIZE)
        return true;
    } else if (angle < 110 && angle > 70) {
      if (x + TILE_SIZE / 2 > region.ur.x * TILE_SIZE)
        return true;
    } else if (angle < 200 && angle > 160) {
      if (y - TILE_SIZE / 2 < region.ll.y * TILE_SIZE)
        return true;
    } else if (angle < 290 && angle > 250) {
      if (x - TILE_SIZE / 2 < region.ll.x * TILE_SIZE)
        return true;
    }
    return false;
  }


  /**
   * Method to compute the coordinates of the tunnel that the robot moves to
   * 
   * @param tunnel
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
      if (horizontalTunnel(tunnel)) {
        tunnelStartX -= 0.5;
      } else
        tunnelStartY -= 0.5;
    } else {
      tunnelStartX = x1;
      tunnelStartY = y1;
      tunnelEndX = x0;
      tunnelEndY = y0;

      tunnelTheta += 180;
      if (horizontalTunnel(tunnel)) {
        tunnelStartX += 0.5;
      } else
        tunnelStartY += 0.5;
    }

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
   * Method for device to travel back to its corner
   * 
   * @param corner, int given by wifi parameter
   */
  public static void navigateToStart(int corner) {
    double x, y;
    if (corner == 0) {
      x = 0;
      y = 0;
    } else if (corner == 1) {
      x = 15;
      y = 0;
    } else if (corner == 2) {
      x = 15;
      y = 9;
    } else {
      x = 0;
      y = 9;
    }
    Navigation.travelTo(x, y);
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
   * Returns of device's bin
   * 
   * @return Point of bin location of device's team color
   */
  public static Point getBin() {
    if (redTeam == TEAM_NUMBER) {
      return redBin;
    } else
      return greenBin;
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
