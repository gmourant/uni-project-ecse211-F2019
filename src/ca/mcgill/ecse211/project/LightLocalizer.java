package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * Light localization that assumes that the device is facing NORTH and is on a 45 degree line in between (0,0) and (1,1)
 * 
 * @author Aakarsh
 * @author Steven
 * @author Hassan
 */

public class LightLocalizer {
  private static final double LIGHTSENSOR_DELTA = 30;
  long correctionStart, correctionEnd;
  private SampleProvider light = colorSensor.getRedMode();
  private float[] lightData = new float[colorSensor.sampleSize()];
  private int lightValue;
  // Our device has a 6.4 cm distance between it's center and the light sensor
  // The light sensor is placed at the front
  private double offSet = 6.4;
  private int INITIAL_LIGHT;

  /**
   * @return returns true if black line detected, else false
   */
  public boolean correctionTrigger() {
    light.fetchSample(lightData, 0);
    lightValue = (int) (lightData[0] * 100);
    if (Math.abs(lightValue - INITIAL_LIGHT) > LIGHTSENSOR_DELTA) {
      Sound.beep();
      return true;
    }
    return false;
  }

  /**
   * main method assuming that the device is facing north turn 45 degree and make sure that we are not already on (1,1)
   * by starting off with a spin
   */
  public void localize() {
    // initial light data that will be used to detect black lines
    light.fetchSample(lightData, 0);
    INITIAL_LIGHT = (int) (lightData[0] * 100);
    /*
     * Correct Y-Axis
     */
    while (!correctionTrigger()) {
      leftMotor.forward();
      rightMotor.forward();
    }
    odometer.setY(TILE_SIZE - offSet);
    leftMotor.rotate(convertDistance(offSet + TILE_SIZE / 2), true);
    rightMotor.rotate(convertDistance(offSet + TILE_SIZE / 2), false);
    /*
     * Turn To X-axis and correct it
     */
    Navigation.turnTo(90 * Math.PI / 180);
    while (!correctionTrigger()) {
      leftMotor.forward();
      rightMotor.forward();
    }
    odometer.setX(TILE_SIZE - offSet);
    leftMotor.rotate(convertDistance(offSet), true);
    rightMotor.rotate(convertDistance(offSet), false);
 
    // face NORTH
    Navigation.turnTo(0);
    leftMotor.stop();
    rightMotor.stop();
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

}
