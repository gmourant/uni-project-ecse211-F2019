package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.utility.Delay;
import lejos.hardware.Sound;

/**
 * Navigates the robot depending on given coordinates avoids obstacles and resumes navigation
 *
 * @author Steven
 * @author Hassan
 * @author Aakarsh
 * @author David
 *
 */

public class Navigation {

  /**
   * tells us whether an obstacle was detected
   */
  public static boolean obstacleDetected = true;

  /**
   * the current x position of the robot
   */
  public static double currentX;

  /**
   * the current y position of the robot
   */
  public static double currentY;

  /**
   * an instance of the navigation
   */
  private static Navigation nav; // Returned as singleton

  /**
   * Orientates robot towards desired destination rotates forward to the coordinate and resumes when avoided
   *
   * @param x x coordinate
   * @param y y coordinate
   */

  public static void travelTo(double x, double y) {
    // reset and initiliaze motors
    currentX = x;
    currentY = y;
    leftMotor.stop(true);
    rightMotor.stop(false);
    launchMotor.stop();
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);

    obstacleDetected = false;
    x = x * TILE_SIZE;
    y = y * TILE_SIZE;
    // Calculate x & y trajectory
    double dx = x - odometer.getXYT()[0];
    double dy = y - odometer.getXYT()[1];
    // Calculate desired angle to turn to in relation to current angle
    double angle = Math.atan2(dx, dy);

    turnTo(angle);
    // Calculate absolute trajectory
    double distance = Math.hypot(dx, dy);

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(distance), true);
    rightMotor.rotate(convertDistance(distance), false);
  }

  public static void travelTo(double x, double y, Point bin) {
    // reset and initiliaze motors
    currentX = x;
    currentY = y;
    leftMotor.stop(true);
    rightMotor.stop(false);
    launchMotor.stop();
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);

    obstacleDetected = false;
    x = x * TILE_SIZE;
    y = y * TILE_SIZE;
    // Calculate x & y trajectory
    double dx = x - odometer.getXYT()[0];
    double dy = y - odometer.getXYT()[1];
    // Calculate desired angle to turn to in relation to current angle
    double angle = Math.atan2(dx, dy);

    turnTo(angle);
    // Calculate absolute trajectory
    double distance = Math.hypot(dx, dy);

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(distance), true);
    rightMotor.rotate(convertDistance(distance), true);

    // continuously check for obstacles
    do {
      if (ObstacleAvoidance.checkForObstacle()) {
        // stop moving
        leftMotor.stop(true);
        rightMotor.stop(false);
        // Sound.beepSequenceUp();
        // obstacleDetected = true;
      }
    } while (leftMotor.isMoving() || rightMotor.isMoving());

    if (ObstacleAvoidance.obstacleDetected) {
      double[] pos = odometer.getXYT();
      ObstacleAvoidance.avoidObstacle(pos[0], pos[1]);
      if (!ObstacleAvoidance.obstacleDetected) {
        Main.navigateToLaunchPosition(bin);
      }
    } 
    /*
    else {
      Main.navigateToLaunchPosition(bin);
    }
    */
    // double[] pos = odometer.getXYT();
    // double[] lp = Navigation.launchPosition(bin.x, bin.y, RADIUS, pos[0], pos[1]);
    // if (!Main.validPoint(lp[0], lp[1], island.ll.x, island.ll.y, island.ur.x, island.ur.y)) {
    // lp = Main.navigateToAlternateLaunchPosition(lp, bin);
    // }
    // Navigation.turnTo(Main.findAngle(odometer.getXYT()[0], odometer.getXYT()[1], bin.x, bin.y));
    // Navigation.travelTo(lp[0] / TILE_SIZE, lp[1] / TILE_SIZE, bin);
  }


  /**
   * Orients robot towards desired destination rotates forward to the coordinate until it reaches the circle of radius r
   * from the given point avoids obstacle and resumes when avoided.
   *
   * @param x x coordinate
   * @param y y coordinate
   * @param r radius
   */
  public static void travelTo(double x, double y, double r) {
    // reset and initiliaze motors
    currentX = x;
    currentY = y;
    leftMotor.stop(true);
    rightMotor.stop(false);
    // launchMotor.stop();
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);

    obstacleDetected = false;
    x = x * TILE_SIZE;
    y = y * TILE_SIZE;
    // Calculate x & y trajectory
    double dx = x - odometer.getXYT()[0];
    double dy = y - odometer.getXYT()[1];
    // Calculate desired angle to turn to in relation to current angle
    double angle = Math.atan2(dx, dy);

    // Calculate absolute trajectory

    double distance = Math.hypot(dx, dy);
    if (distance - r >= 0) {
      turnTo(angle);
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      sleep(50);
      leftMotor.rotate(convertDistance(distance - r), true);
      rightMotor.rotate(convertDistance(distance - r), false);
    } else {
      turnTo(angle + Math.toRadians(180));
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      sleep(50);
      leftMotor.rotate(convertDistance(Math.abs(distance - r)), true);
      rightMotor.rotate(convertDistance(Math.abs(distance - r)), false);
    }

    dx = x - odometer.getXYT()[0];
    dy = y - odometer.getXYT()[1];
    angle = Math.atan2(dx, dy);
    turnTo(angle);
    // TODO obstacle avoidance
  }

  /**
   * finds the ideal launch position to launch a projectile r distance away from given point
   *
   * @param x coordinates of x
   * @param y coordinates of y
   * @param r radius of position
   */
  public static double[] launchPosition(double x, double y, double r, double curr_x, double curr_y) {
    x = x * TILE_SIZE;
    y = y * TILE_SIZE;
    double dx = x - curr_x;
    double dy = y - curr_y;
    // Calculate desired angle to turn to in relation to current angle
    double angle = Math.atan2(dx, dy);
    // Calculate desired distance to be covered
    double distance = Math.hypot(dx, dy) - r;
    // Calculate absolute trajectory
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    double[] pos = new double[2];
    pos[0] = curr_x + distance * Math.sin(angle);
    pos[1] = curr_y + distance * Math.cos(angle);
    return pos;
  }

  /**
   * finds an alternate launch position to launch a projectile r distance away from given point. This is required in
   * case the previously computed ideal launch point is not on the island.
   *
   * @param x coordinates of x
   * @param y coordinates of y
   * @param r radius of position
   * @param lp previously computed launch point
   * @return
   */
  static double dummy_x = odometer.getXYT()[0];
  static double dummy_y = odometer.getXYT()[1];

  public static double[] alternateLaunchPosition(Point bin, double r, double[] lp) {
    double x = bin.x * TILE_SIZE;
    double y = bin.y * TILE_SIZE;

    double dx = x - dummy_x;
    double dy = y - dummy_y;

    double angle = Math.atan2(dx, dy) + 90;

    // move clockwise along the circle
    dummy_x += DUMMY_DISTANCE * Math.sin(angle);
    dummy_y += DUMMY_DISTANCE * Math.cos(angle);

    lp = launchPosition(bin.x, bin.y, RADIUS, dummy_x, dummy_y);

    return lp;
  }

  /**
   * turns toward desired angle
   *
   * @param theta theta in radians
   */
  public static void turnTo(double theta) {
    double thetaDegree = theta * 180 / Math.PI;
    double angle = thetaDegree - odometer.getXYT()[2];
    leftMotor.stop(true);
    rightMotor.stop(false);
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    // remove negative angles
    if (angle < 0)
      angle += 360;
    // minimal angle
    angle = angle % 360;
    sleep(50);
    if (angle > 180) {
      leftMotor.rotate(-convertAngle(360 - angle), true);
      rightMotor.rotate(convertAngle(360 - angle), false);
    } else {
      leftMotor.rotate(convertAngle(angle), true);
      rightMotor.rotate(-convertAngle(angle), false);
    }
    sleep(50);
  }

  /**
   * Device travels by half a tile either by backing up or going forward
   *
   * @param back, if true device goes backward, else forward
   */
  public static void goMid(boolean back) {
    leftMotor.stop(true);
    rightMotor.stop(false);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    sleep(100);
    if (back) {
      leftMotor.rotate(convertDistance(-TILE_SIZE / 2), true);
      rightMotor.rotate(convertDistance(-TILE_SIZE / 2), false);
    } else {
      leftMotor.rotate(convertDistance(TILE_SIZE / 2), true);
      rightMotor.rotate(convertDistance(TILE_SIZE / 2), false);
    }

  }

  /**
   * true when no obstacle detected and motors are moving
   *
   * @return boolean
   */

  public boolean isNavigating() {
    return !obstacleDetected;
  }

  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance. From Lab 2
   * SquareDriver.java
   *
   * @param distance
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that angle. From Lab 2
   * SquareDriver.java
   *
   * @param angle
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * TRACK * angle / 360.0);
  }

  /**
   * Returns the Navigation Object. Use this method to obtain an instance of Navigation.
   *
   * @return the Navigation Object
   */
  public synchronized static Navigation getNavigation() {
    if (nav == null) {
      nav = new Navigation();
    }
    return nav;
  }

  /**
   * Thread sleeps in ms for threads to catch up
   *
   * @param time, time in ms
   */
  private static void sleep(int time) {
    try {
      Thread.sleep(time);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

}
