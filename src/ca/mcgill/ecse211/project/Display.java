package ca.mcgill.ecse211.project;

import java.text.DecimalFormat;

import lejos.hardware.Button;
//static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.project.Resources.*;;

/**
 * This class is used to display the content of the odometer variables (x, y, Theta)
 */
public class Display implements Runnable {

  /**
   * holds the current position of the robot
   */
  private double[] position;
  
  /**
   * time period for the display
   */
  private final long DISPLAY_PERIOD = 25;
  
  /**
   * length of timeout of the display
   */
  private long timeout = Long.MAX_VALUE;
  
  /**
   * Starts running the display. Constantly prints the current position of the robot, ie, the odometer reading
   */
  public void run() {
    
    LCD.clear();
    
    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();

      // Retrieve x, y and Theta information
      position = odometer.getXYT();
      
      // Print x,y, and theta information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
//      LCD.drawString("X: " + numberFormat.format(position[0]), 0, 0);
//      LCD.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
//      LCD.drawString("T: " + numberFormat.format(position[2]), 0, 2);
      
//      System.out.println("***Position***");
//      System.out.println("X: " + numberFormat.format(position[0]));
//      System.out.println("Y: " + numberFormat.format(position[1]));
//      System.out.println("T: " + numberFormat.format(position[2]) + "\n");
      
      // this ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);

  }
  
  /**
   * Sets the timeout in ms.
   * 
   * @param timeout
   */
  public void setTimeout(long timeout) {
    this.timeout = timeout;
  }
  
  /**
   * Shows the text on the LCD, line by line.
   * 
   * @param strings comma-separated list of strings, one per line
   */
  public static void showText(String... strings) {
    LCD.clear();
    for (int i = 0; i < strings.length; i++) {
      LCD.drawString(strings[i], 0, i);
    }
  }
  
  /**
   * start menu when program is uploaded and run from EV3Brick
   * 
   */
  
  public static void printMainMenu() {
    LCD.clear(); // the screen at initialization
    LCD.drawString("Press any button", 0, 0);
    LCD.drawString("to continue", 0, 1);
    Button.waitForAnyPress();
    Main.sleepFor(100);
  }
}