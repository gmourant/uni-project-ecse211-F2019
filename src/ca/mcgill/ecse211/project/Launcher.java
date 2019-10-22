package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;

/**
 * This controls the launching mechanism.
 * 
 * @author Steven
 * @author Aakarsh
 */
public class Launcher {
  /**
   * Runs the launching sequence. It waits 5 seconds for manual reloading and then launches
   */
  public static void launch() {
    while (true) {
      // wait for reloading
      Main.sleepFor(5000);
      
      // launch
      launchMotor.setAcceleration(100);
      launchMotor.rotate(360);

      // stop for reloading
      launchMotor.stop();
    }
  }
}
