package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.Sound;

/**
 * 
 * 
 * @author Hassan, Aakarsh, Steven
 *
 */
public class ObstacleAvoidance {
  public static boolean obstacleDetected;
  private static boolean rightIsSafe;
  private static boolean leftIsSafe;
  private static double leftAngle;
  private static double rightAngle;
  private static double obstacleDetectionDistance;

  /**
   * checks if there is an obstacle in front of the robot
   * 
   * @return true if there is an obstacle, false if there isn't
   */
  public static boolean checkForObstacle() {
    float distance = Main.UP.getDistance();
    obstacleDetected = distance < THRESHOLD;
    if (obstacleDetected) {
      obstacleDetectionDistance = distance;
    }
    return obstacleDetected;
  }

  /**
   *
   * @param currentX
   * @param currentY
   */
  public static void avoidObstacle(double currentX, double currentY) {
    leftMotor.stop(true);
    rightMotor.stop(false);
    //
    usMotor.setSpeed(40);

    // prepare usMotor for sweep
    usMotor.resetTachoCount();
    // Rotate sensor motor to the right (clockwise) until it detects a rising edge
    while ((Main.UP.getDistance() < THRESHOLD) && Math.abs(usMotor.getTachoCount()) <= US_MOTOR_LIMIT) {
      usMotor.backward();
    }
    usMotor.stop(false);
    Sound.beep();
    rightAngle = usMotor.getTachoCount(); // angle at which first rising edge is detected
    if (rightAngle < US_MOTOR_LIMIT) {
      rightIsSafe = true;
    }
    // rotate sensor in the opposite direction until obstacle can be seen again
    while (Main.UP.getDistance() > THRESHOLD) {
      usMotor.forward();
    }


    // Rotate sensor motor to the left until it detects a rising edge
    while ((Main.UP.getDistance() < THRESHOLD) && Math.abs(usMotor.getTachoCount()) <= US_MOTOR_LIMIT) {
      usMotor.forward();
    }
    leftAngle = Math.abs(usMotor.getTachoCount());
    usMotor.stop(false);
    Sound.beep();
    if (leftAngle < US_MOTOR_LIMIT) {
      leftIsSafe = true;
    }

    // recenter sensor motor
    usMotor.rotateTo(0);

    // TODO: Try deciding to go where there is the smaller angle
    // HARD CODED AVOIDANCE
    // might replace with bangbang control
    Point left = calcDisplacementPoint(TILE_SIZE + TRACK / 2, -90);
    Point right = calcDisplacementPoint(TILE_SIZE + TRACK / 2, 90);
    if (!Main.validPoint(right.x, right.y, island))
      rightIsSafe = false;
    if (!Main.validPoint(left.x, left.y, island))
      leftIsSafe = false;
    if (rightIsSafe) {
      takeRightPath(right);
    }

    else if (leftIsSafe) {
      takeLeftPath(left);
    } else {

    }
    // } else {
    // if (rightIsSafe) {
    // takeRightPath(right);
    // } else if (leftIsSafe) {
    // takeLeftPath(left);
    // }
    // }


    // bangbangAvoid(currentX, currentY);
    obstacleDetected = false;

  }

  /**
   * 
   * @param currentX
   * @param currentY
   */
  private static void bangbangAvoid(double currentX, double currentY) {
    // set wheels perpendicular from obstacle
    leftMotor.stop();
    rightMotor.stop();
    Sound.twoBeeps();
    double dx, dy, angle, theta;

    // Clockwise bang bang
    if (rightIsSafe) {
      usMotor.rotateTo(45);
      Navigation.turnTo(Math.toRadians(odometer.getXYT()[2] + 90));
      while (true) {
        dx = currentX * TILE_SIZE - odometer.getXYT()[0];
        dy = currentY * TILE_SIZE - odometer.getXYT()[1];
        angle = Math.toDegrees(Math.atan2(dx, dy));
        theta = odometer.getXYT()[2];
        // angle correction when atan2 gives a negative number
        // if (angle < 0) {
        // // Angle correction when Pointing south
        // if (angle <= -90) {
        // angle = (Math.abs(angle - 90)) % 360;
        // }
        // // Angle correction when pointing east
        // else if (angle > -100)
        // angle = (Math.abs(angle - 270)) % 360;
        // }

        if (angle < 0)
          angle += 360;
        // minimal angle
        angle = angle % 360;

        // make sure that new angle is within range
        // Exits when robot's internal odometer angle points toward desired coordinate
        if (Math.abs(odometer.getXYT()[2] - angle) < THRESHOLD_RANGE)
          break;
        // BangBangController
        Main.distance.fetchSample(Main.sampleUS, 0);
        int distance = (int) (Main.sampleUS[0] * 100.0);
        int error = BAND_CENTER - distance;
        sleep(25);
        if (Math.abs(error) <= BAND_WIDTH) {
          leftMotor.setSpeed(MOTOR_HIGH);
          rightMotor.setSpeed(MOTOR_HIGH);
          rightMotor.forward();
          leftMotor.forward();
        } else if (error > 0) { // too close to wall
          leftMotor.setSpeed(MOTOR_HIGH);
          rightMotor.setSpeed(MOTOR_LOW);
          rightMotor.forward();
          leftMotor.backward();
        } else if (error < 0) { // too far from wall
          leftMotor.setSpeed(MOTOR_LOW);
          rightMotor.setSpeed(MOTOR_HIGH);
          leftMotor.forward();
          rightMotor.forward();
        }
      }
    }

    // counter-clockwise bangbang
    else if (leftIsSafe) {
      usMotor.rotateTo(-45);
      sleep(50);
      Navigation.turnTo((odometer.getXYT()[2] - 90) * Math.PI / 180);
      while (true) {
        dx = currentX * TILE_SIZE - odometer.getXYT()[0];
        dy = currentY * TILE_SIZE - odometer.getXYT()[1];
        angle = (Math.atan2(dx, dy) * 180 / Math.PI);
        theta = odometer.getXYT()[2];
        // angle correction when atan2 gives a negative number
        // if (angle < 0) {
        // // Angle correction when Pointing south
        // if (angle <= -90) {
        // angle = (Math.abs(angle - 90)) % 360;
        // }
        // // Angle correction when pointing east
        // else if (angle > -100)
        // angle = (Math.abs(angle - 270)) % 360;
        // }

        if (angle < 0)
          angle += 360;
        // minimal angle
        angle = angle % 360;

        // make sure that new angle is within range
        // Exits when robot's internal odometer angle points toward desired coordinate
        if (Math.abs(odometer.getXYT()[2] - angle) < THRESHOLD_RANGE)
          break;
        // BangBangController
        Main.distance.fetchSample(Main.sampleUS, 0);
        int distance = (int) (Main.sampleUS[0] * 100.0);
        int error = BAND_CENTER - distance;
        sleep(25);
        if (Math.abs(error) <= BAND_WIDTH) {
          leftMotor.setSpeed(MOTOR_HIGH);
          rightMotor.setSpeed(MOTOR_HIGH);
          rightMotor.forward();
          leftMotor.forward();
        } else if (error > 0) { // too close to wall
          leftMotor.setSpeed(MOTOR_LOW);
          rightMotor.setSpeed(MOTOR_HIGH);
          rightMotor.forward();
          leftMotor.backward();
        } else if (error < 0) { // too far from wall
          leftMotor.setSpeed(MOTOR_HIGH);
          rightMotor.setSpeed(MOTOR_LOW);
          leftMotor.forward();
          rightMotor.forward();
        }
      }
    }
    Sound.beep();
    leftMotor.stop(true);
    rightMotor.stop(false);
    usMotor.rotateTo(0);
  }

  /**
   *
   * @param x
   * @param y
   */
  private static void takeRightPath(Point p) {
    // // make a 90 degree turn to the Right (clockwise)
    // Navigation.turnTo(Math.toRadians(odometer.getXYT()[2] + 90));
    // // move forward obstacle size
    // rightMotor.rotate(Navigation.convertDistance(TILE_SIZE), true);
    // leftMotor.rotate(Navigation.convertDistance(TILE_SIZE), false);
    // // rotate back to original orientation
    // Navigation.turnTo(Math.toRadians(odometer.getXYT()[2] - 90));
    //
    // rightMotor.rotate(Navigation.convertDistance(TILE_SIZE + THRESHOLD), true);
    // leftMotor.rotate(Navigation.convertDistance(TILE_SIZE + THRESHOLD), false);

    Navigation.travelTo(p.x, p.y, Main.getBin());
    Point nextPoint = calcDisplacementPoint(TILE_SIZE + THRESHOLD, 90);
    Navigation.travelTo(nextPoint.x, nextPoint.y, Main.getBin());

  }


  private static void takeLeftPath(Point p) {
    // // make a 90 degree turn to the left (counter-clockwise)
    // Navigation.turnTo((odometer.getXYT()[2] - 90) * Math.PI / 180);
    // // move forward obstacle size
    // rightMotor.rotate(Navigation.convertDistance(TILE_SIZE), true);
    // leftMotor.rotate(Navigation.convertDistance(TILE_SIZE), false);
    // // rotate back to original orientation
    // Navigation.turnTo((odometer.getXYT()[2] + 90) * Math.PI / 180);
    //
    // rightMotor.rotate(Navigation.convertDistance(TILE_SIZE + THRESHOLD), true);
    // leftMotor.rotate(Navigation.convertDistance(TILE_SIZE + THRESHOLD), false);

    Navigation.travelTo(p.x, p.y, Main.getBin());
    Point nextPoint = calcDisplacementPoint(TILE_SIZE + THRESHOLD, 90);
    Navigation.travelTo(nextPoint.x, nextPoint.y, Main.getBin());
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

  private static Point calcDisplacementPoint(double displacementDistance, double displacementAngle) {
    double x = odometer.getXYT()[0]
        + (displacementDistance) * Math.cos(Math.toRadians(odometer.getXYT()[2] + displacementAngle));
    double y = odometer.getXYT()[1]
        + (displacementDistance) * Math.sin(Math.toRadians(odometer.getXYT()[2] + displacementAngle));
    return new Point(x / TILE_SIZE, y / TILE_SIZE);
  }
}
