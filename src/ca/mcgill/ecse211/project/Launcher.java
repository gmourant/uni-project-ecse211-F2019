package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;;

public class Launcher {
  // setup launching mechanism
  public static void launch() {
    while(true) {
      Main.sleepFor(5000);
      
      //launch
      launchMotor.setAcceleration(100);
      launchMotor.rotate(360);
      
      //stop for reloading
      launchMotor.stop();
    }
  }
}

