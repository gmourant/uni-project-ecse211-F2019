package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;;

public class Launcher {
  // setup launching mechanism
  public static void launch() {
    launcherMotor.setAcceleration(10000);
    launcherMotor.forward();
  }
}
