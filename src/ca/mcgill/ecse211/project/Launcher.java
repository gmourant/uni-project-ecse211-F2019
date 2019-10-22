package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import java.io.File;
import lejos.hardware.Sound;;

/**
 * 
 * @author Steven
 *@author Aakarsh
 */
public class Launcher {
  // setup launching mechanism
  public static void launch() {
    while (true) {
      Main.sleepFor(5000);
      // Sound.playSample(new File("Kobe (1).wav"), Sound.VOL_MAX);
      // launch
      launchMotor.setAcceleration(100);
      launchMotor.rotate(360);

      // stop for reloading
      launchMotor.stop();
    }
  }
}
