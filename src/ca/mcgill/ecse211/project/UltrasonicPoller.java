package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.FILTER_OUT;
import lejos.robotics.SampleProvider;

/**
 * Simple UltrasonicPoller that samples the US sensor 
 * and method returns true
 * when obstacle is detected
 * 
 * @author Aakarsh 
 * @author Hassan
 * 
 */
public class UltrasonicPoller {
  private float[] sampleUS;
  SampleProvider distance;
  int wallDistance;
  int filterControl;
  
  public UltrasonicPoller(SampleProvider distance, float[] sampleUS) {
    this.distance = distance;
    this.sampleUS = sampleUS;
  }
  
  /**
   * 
   * @return float distance in cm
   */
  public float getDistance(){
    distance.fetchSample(sampleUS, 0);
    wallDistance = (int) (sampleUS[0] * 100.0); // extract from buffer, convert to cm, cast to int
    try {
        Thread.sleep(80);
    } catch (InterruptedException e) {}
    if(sampleUS[0]>255){
        wallDistance = 255;
    }
    return filter(wallDistance);
}
  /**
   * Ultrasonic sensor filter
   * 
   * @param d distance read by ultrasonic sensor in cm
   * @return filtered distance in cm
   */
  public float filter(float d) {
    if (d >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the filter value
      filterControl++;
      return filter(d);
    } else if (d >= 255) {
      // Repeated large values, so there is nothing there: leave the distance alone
      return d;
    } else {
      // distance went below 255: reset filter and leave distance alone.
      filterControl = 0;
      return d;
    }
  }
}