package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.sensor.EV3UltrasonicSensor;
//import lejos.robotics.SampleProvider;

/**
 * Contains the implementation of Ultrasonic Localization and 
 * abstracts the logic to behind it to one method call, which is localize.
 * It assumes that the robot placed along an imaginary 45 degree line 
 * between (0,0) and (1,1) on a tiled course. It reorients the robot
 * to face approximately NORTH, and sets the Theta of Odometetry to 0. 
 * @author Steven
 * @author Hassan
 * @author Aarkash
 * @version 1.2.1
 * @since 1.1.1
 */
public class UltrasonicLocalizer {

  private final int type;
//  private float[] usData;                 // not needed
//  SampleProvider distance;                        
//  private final EV3UltrasonicSensor us;
  private static int wallDistance = 30;
  private static int noiseMargin = 2;
  int filterControl;
  double angle1, angle2, dAngle;


  /**
   * Constructor of UltrasonicLocalizer
   * @param 0 for falling edge wall detection, 1 for rising edge
   * @param EV3Ultrasonic distance mode. Should be distance US_SENSOR 
   */
  public UltrasonicLocalizer(int type, EV3UltrasonicSensor us) {
    this.type = type;
//    this.us = us;
  }


  /**
   * Orients the robot to face north and sets the Theta value of Odometer to
   * approximately 0.
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
  }

}
