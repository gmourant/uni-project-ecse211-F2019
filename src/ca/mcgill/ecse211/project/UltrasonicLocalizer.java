package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.sensor.EV3UltrasonicSensor;
//import lejos.robotics.SampleProvider;

/**
 * Given that the system is placed along an
 * imaginary 45 degree line between (0,0) and (1,1)
 * reorient the system facing NORTH
 * 
 * @author Hassan
 * @author Aakarsh
 * @author Steve
 *
 */

public class UltrasonicLocalizer {

  private final int type;
//  private float[] usData;                 // not needed
//  SampleProvider distance;                        
//  private final EV3UltrasonicSensor us;
  private static int wallDistance = 30;
  private static int noiseMargin = 1;
  int filterControl;
  double angle1, angle2, dAngle;


  public UltrasonicLocalizer(int type, EV3UltrasonicSensor us) {
    this.type = type;
//    this.us = us;
  }


/**
 * 
 */
  public void localize() {
    //initialize motors
    leftMotor.stop(true);
    rightMotor.stop(false);
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);
    leftMotor.setSpeed(US_ROTATE_SPEED);
    rightMotor.setSpeed(US_ROTATE_SPEED);
    if (type == 0) {
      fallingEdge();
    } 
    //wrap around 359 degree to 0 degree
    if (angle1 > angle2) {
      dAngle = 45 - (angle1 + angle2) / 2;
    } else if (angle1 < angle2) {
      dAngle = 225 - (angle1 + angle2) / 2;
    } else
      dAngle = angle1 + angle2 / 2;
    //update current odometer angle
    odometer.update(0, 0, dAngle);
    navigate.turnTo(0);
  }
  
  /**
   * Record angle and switch direction when wall is detected
   * Initializes robot to start facing the void
   */
  public void fallingEdge() {
    // turn the system until the sensor observes nothing 
    while (Main.UP.getDistance() < wallDistance + noiseMargin) {
      leftMotor.forward();
      rightMotor.backward();
    }
    //turn the system until it sees the back wall
    while (Main.UP.getDistance() > wallDistance - noiseMargin) {
      leftMotor.forward();
      rightMotor.backward();
    }
    //angle at which the sensor sees the back wall
    angle1 = odometer.getXYT()[2];
    //turn the system in the opposite direction until the sensor does not see the wall
    while (Main.UP.getDistance() < wallDistance + noiseMargin) {
      leftMotor.backward();
      rightMotor.forward();
    }
    // turn the system until it sees the left wall 
    while (Main.UP.getDistance() > wallDistance - noiseMargin) {
      leftMotor.backward();
      rightMotor.forward();
    }
    //angle at which the sensor sees the wall  
    angle2 = odometer.getXYT()[2];
    leftMotor.stop(true);
    rightMotor.stop(false);
  }

}
