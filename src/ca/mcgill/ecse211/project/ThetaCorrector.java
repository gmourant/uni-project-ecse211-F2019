/**
 * 
 */
package ca.mcgill.ecse211.project;

/**
 * @author Hassan
 * @version 1.1.1
 * @since 1.1.1
 * 
 * This class contains the method to correct the orientation of the robot after
 * ultrasonic localization. The logic of the correction is in the method correctTheta().
 * When the method is called, the robot moves forward until a light sensor detects a black line. 
 * When this happens, the associated wheel is stopped, until the other wheel/light sensor catches up.
 * After that, orientation is either set to 0, 90, 180, or 270, depending on the intial value of theta
 * 
 */
public class ThetaCorrector extends LightLocalizer {

  
  /**
   * corrects the orientation of the robot.
   * Assumes that robot is moving forward.
   */
  void correctTheta() {
    
  };
  
  
}
