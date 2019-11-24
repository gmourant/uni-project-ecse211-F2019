package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * Contains the implementation of Light Localization and 
 * abstracts the logic behind it to one method call, which is localize.
 * It assumes that the device is facing NORTH and is
 * on a 45 degree line in between (0,0) and (1,1), i.e. it is being 
 * called after Ultrasonic Localization. Use the localize method
 * to perform localization.
 * 
 * @author Aakarsh
 * @author Steven
 * @author Hassan
 * @version 1.2.1
 * @since 1.1.1
 */
public class LightLocalizer {
  private static final double LIGHTSENSOR_DELTA = 20;
  long correctionStart, correctionEnd;

  private SampleProvider leftLight = leftColorSensor.getRedMode();
  private float[] leftLightData = new float[leftColorSensor.sampleSize()];
  private int oldLeftLightValue;
  private boolean leftDetects = false;


  private SampleProvider rightLight = rightColorSensor.getRedMode();
  private float[] rightLightData = new float[rightColorSensor.sampleSize()];
  private int oldRightLightValue;
  private boolean rightDetects = false;

  // Our device has a 6.4 cm distance between it's center and the light sensor
  // The light sensor is placed at the front
  // private double offSet = 6.4;
  private double offSet = 4;
  private int rightLightValue, leftLightValue;

  /**
   * fetches a light sample and compares it with the initial
   * light sample value of the left light sensor
   * @return returns true if black line detected, else false
   */
  public boolean leftCorrectionTrigger() {
    leftLight.fetchSample(leftLightData, 0);
    leftLightValue = (int) (leftLightData[0] * 1000);
    System.out.println(leftLightValue);

    if (oldLeftLightValue - leftLightValue > LIGHTSENSOR_DELTA) {
      leftMotor.stop(true);
      rightMotor.stop(false);
      //Sound.beep();
      leftDetects = true;
      return true;
    }
    oldLeftLightValue = leftLightValue;
    leftDetects = false;
    return false;
  }

  /**
   * fetches a light sample and compares it with the initial
   * light sample value of the right light sensor
   * @return returns true if black line detected, else false
   */
  public boolean rightCorrectionTrigger() {
    rightLight.fetchSample(rightLightData, 0);
    rightLightValue = (int) (rightLightData[0] * 1000);
    System.out.println(rightLightValue);
    if (oldRightLightValue - rightLightValue > LIGHTSENSOR_DELTA) {
      leftMotor.stop(true);
      rightMotor.stop(false);
      //Sound.buzz();
      rightDetects = true;
      return true;
    }
    oldRightLightValue = rightLightValue;
    rightDetects = false;
    return false;
  }

  /**
   * Implements the logic of light localization.
   * Sets the values of (X,Y) of the Odometer correctly, and 
   * corrects the orientation of the robot.
   */
  public void localize() {
    leftMotor.setSpeed(MOTOR_NORMAL);
    rightMotor.setSpeed(MOTOR_NORMAL);
    // move robot forward until one sensor sees a line
    leftMotor.forward();
    rightMotor.forward();
    while (!leftCorrectionTrigger() && !rightCorrectionTrigger());
    leftMotor.stop(true);
    rightMotor.stop(false);
    
    sleep(100);
    
    correctTheta(0,true);

    odometer.setY(TILE_SIZE + offSet);
    
    leftMotor.rotate(convertDistance(-offSet), true);
    rightMotor.rotate(convertDistance(-offSet), false);

    sleep(100);
    
    // Turn To X-axis and correct it
    Navigation.turnTo(Math.toRadians(90));
    leftMotor.stop(true);
    rightMotor.stop(false);
    leftMotor.setSpeed(MOTOR_NORMAL);
    rightMotor.setSpeed(MOTOR_NORMAL);

    sleep(100);
    
    // move robot forward until one sensor sees a line
    leftMotor.forward();
    rightMotor.forward();
    while (!leftCorrectionTrigger() && !rightCorrectionTrigger());
    leftMotor.stop(true);
    rightMotor.stop(false);

    sleep(100);
    
    correctTheta(90,true);
    
    sleep(100);
    
    odometer.setX(TILE_SIZE + offSet);
    
    leftMotor.rotate(convertDistance(-offSet), true);
    rightMotor.rotate(convertDistance(-offSet), false);

    sleep(100);

  }
  
  public void localizeForward(double angle) {
    localizeForward(angle, false);
  }
  public void localizeForward(double angle, boolean backup) {
    leftMotor.stop(true);
    rightMotor.stop(false);
    leftMotor.setSpeed(MOTOR_NORMAL);
    rightMotor.setSpeed(MOTOR_NORMAL);
    
    sleep(100);
    
    // move robot forward until one sensor sees a line
    leftMotor.forward();
    rightMotor.forward();
    while (!leftCorrectionTrigger() && !rightCorrectionTrigger());
    leftMotor.stop(true);
    rightMotor.stop(false);

    sleep(100);
    
    correctTheta(angle,backup);
    
    leftMotor.setSpeed(MOTOR_NORMAL);
    rightMotor.setSpeed(MOTOR_NORMAL);
    sleep(100);
    leftMotor.rotate(convertDistance(-offSet), true);
    rightMotor.rotate(convertDistance(-offSet), false);
  }

  /**
   * corrects the orientation of the robot during light localization if left sensor detects first, move right motor
   * until it catches up if right sensor detects first, move left motor until it catches up set odometer angle
   * 
   * @param angle set in the odometer
   */
  private void correctTheta(double angle, boolean backup) {
    // if left sensor detects first, move right motor until it catches up
    leftMotor.stop(true);
    rightMotor.stop(false);
//    leftMotor.setSpeed(MOTOR_NORMAL);
//    rightMotor.setSpeed(MOTOR_NORMAL);
    leftMotor.setSpeed(MOTOR_LOW);
    rightMotor.setSpeed(MOTOR_LOW);
    sleep(100);
    if (leftDetects && !rightDetects) {
      sleep(100);
      if(backup) {
        rightMotor.backward();
        Delay.msDelay(400);
      }
      leftMotor.stop(true);
      rightMotor.stop(false);
      sleep(100);
      rightMotor.forward();
      while (!rightCorrectionTrigger());
      rightMotor.stop();
    }
    // if right sensor detects first, move left motor until it catches up
    else if (!leftDetects && rightDetects) {
      sleep(100);
      if(backup) {
        leftMotor.backward();
        Delay.msDelay(400);
      }
      leftMotor.stop(true);
      rightMotor.stop(false);
      sleep(100);
      leftMotor.forward();
      while (!leftCorrectionTrigger());
      leftMotor.stop();
    }
    // odometer.setTheta(angle);
    leftMotor.stop(true);
    rightMotor.stop(false);
    leftMotor.setSpeed(MOTOR_NORMAL);
    rightMotor.setSpeed(MOTOR_NORMAL);
    sleep(100);
    odometer.setTheta(angle);
  }
  


  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that angle.
   * 
   * @param angle
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * TRACK * angle / 360.0);
  }
  
  private static void sleep(int time) {
    try {
      Thread.sleep(time);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

}
