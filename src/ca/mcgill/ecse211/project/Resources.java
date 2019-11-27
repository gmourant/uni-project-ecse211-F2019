package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.wificlient.WifiConnection;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import java.math.BigDecimal;
import java.util.Map;

/**
 * 
 * This class is used to define static resources in one place for easy access and to avoid cluttering the rest of the
 * codebase.
 * 
 * Integrate this carefully with your existing Resources class (See below for where to add your code from your current
 * Resources file). The order in which things are declared matters!
 *
 * @author Younes Boubekeur
 */
public class Resources {

  // Set these as appropriate for your team and current situation
  /**
   * The default server IP used by the profs and TA's.
   */
  public static final String DEFAULT_SERVER_IP = "192.168.2.3";

  /**
   * The IP address of the server that transmits data to the robot. Set this to the default for the beta demo and
   * competition.
   */
  public static final String SERVER_IP = "192.168.2.31";
  // 192.168.2.29
  // steven's pc: 192.168.2.45
  // david's pc: 192.168.2.63

  /**
   * Your team number.
   */
  public static final int TEAM_NUMBER = 2;

  /**
   * Enables printing of debug info from the WiFi class.
   */
  public static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

  /**
   * Enable this to attempt to receive Wi-Fi parameters at the start of the program.
   */
  public static final boolean RECEIVE_WIFI_PARAMS = true;

  // DECLARE YOUR CURRENT RESOURCES HERE
  // eg, motors, sensors, constants, etc
  /**
   * The tile size in centimeters.
   */
  public static final double TILE_SIZE = 30.48;

  /**
   * The target position for the tile to be shot at
   */
  public static final double[] TARGET_POSITION = {1, 7.5};

  /**
   * the radius of the circle around target
   */
  public static final double RADIUS = 128.016;
  //137.16cm around  4.5 tiles

  /**
   * Offset from the wall (cm).
   */
  public static final int BAND_CENTER = 20;

  /**
   * threshold range.
   */
  public static final double THRESHOLD_RANGE = 7.5;

  /**
   * theshold after which an obstacle is detected
   */
  public static final double THRESHOLD = 25.0;

  /**
   * the maximum angle the sensor motor can turn in degrees.
   */
  public static final double US_MOTOR_LIMIT = 75.0;

  /**
   * Width of dead band (cm).
   */
  public static final int BAND_WIDTH = 10;

  /**
   * The wheel radius in centimeters.
   */
  public static final double WHEEL_RAD = 2.13;

  /**
   * The robot width in centimeters.
   */
  // public static final double TRACK = 11.45;
//  public static final double TRACK = 9.85;
  

  // wheel end to wheel end
  public static final double TRACK = 13.35;

  // wheel center to wheel center
  //public static final double TRACK2 = 13.6;




  /**
   * The speed at which the robot moves forward in degrees per second.
   */
  public static final int FORWARD_SPEED = 250;
  /**
   * Speed of slower rotating wheel (deg/sec).
   */
//  public static final int MOTOR_LOW = 75;
  public static final int MOTOR_LOW = 110;


  /**
   * Speed of slower rotating wheel (deg/sec).
   */
  public static final int MOTOR_NORMAL = 170;

  /**
   * Speed of the faster rotating wheel (deg/sec).
   */
  public static final int MOTOR_HIGH = 210;

  /**
   * The speed at which the robot rotates in degrees per second.
   */
  public static final int ROTATE_SPEED = 125;

  public static final int US_ROTATE_SPEED = 200;


  /**
   * The motor acceleration in degrees per second squared.
   */
  public static final int ACCELERATION = 1000;
     
  /**
   * Timeout period in milliseconds.
   */
  public static final int TIMEOUT_PERIOD = 3000;

  public static final int FILTER_OUT = 30;
  
  /**
   * Dummy distance factor to compute alternative launch points
   */
  public static final double DUMMY_DISTANCE = TILE_SIZE;

  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * The launcher motor.
   */
  public static final EV3LargeRegulatedMotor launchMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

  /**
   * Motor for ultrasonic sensor sweeping.
   */
  public static final EV3MediumRegulatedMotor usMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
  
  /**
   * The ultrasonic sensor.
   */
  public static final EV3UltrasonicSensor US_SENSOR = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
  /**
   * The color sensor.
   */
  public static final EV3ColorSensor rightColorSensor = new EV3ColorSensor(SensorPort.S4);
  public static final EV3ColorSensor leftColorSensor = new EV3ColorSensor(SensorPort.S1);


  /**
   * The LCD.
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();

  /**
   * The odometer.
   */
  public static Odometer odometer = Odometer.getOdometer();

  /**
   * The Navigation.
   */
  public static Navigation navigate = Navigation.getNavigation();

  public static double tunnelStartX; // should be middle of tunnel entry
  public static double tunnelStartY;

  public static double tunnelEndX;
  public static double tunnelEndY;

  public static double tunnelTheta;

  public static boolean notGotWifi = false;

  //////////////////////////////////////

  /**
   * Container for the Wi-Fi parameters.
   */
  public static Map<String, Object> wifiParameters;

  // This static initializer MUST be declared before any Wi-Fi parameters.
  static {
    receiveWifiParameters();
  }

  /**
   * Red team number.
   */
  public static int redTeam = get("RedTeam");

  /**
   * Red team's starting corner.
   */
  public static int redCorner = get("RedCorner");

  /**
   * Green team number.
   */
  public static int greenTeam = get("GreenTeam");

  /**
   * Green team's starting corner.
   */
  public static int greenCorner = get("GreenCorner");

  /**
   * The Red Zone.
   */
  public static Region red = new Region("Red_LL_x", "Red_LL_y", "Red_UR_x", "Red_UR_y");

  /**
   * The Green Zone.
   */
  public static Region green = new Region("Green_LL_x", "Green_LL_y", "Green_UR_x", "Green_UR_y");

  /**
   * The Island.
   */
  public static Region island = new Region("Island_LL_x", "Island_LL_y", "Island_UR_x", "Island_UR_y");

  /**
   * The red tunnel footprint.
   */
  public static Region tnr = new Region("TNR_LL_x", "TNR_LL_y", "TNR_UR_x", "TNR_UR_y");

  /**
   * The green tunnel footprint.
   */
  public static Region tng = new Region("TNG_LL_x", "TNG_LL_y", "TNG_UR_x", "TNG_UR_y");

  /**
   * The location of the target bin.
   */
  public static Point redBin = new Point(get("Red_BIN_x"), get("Red_BIN_y"));

  /**
   * The location of the green target bin.
   */
  public static Point greenBin = new Point(get("Green_BIN_x"), get("Green_BIN_y"));

  /**
   * Receives Wi-Fi parameters from the server program.
   */
  public static void receiveWifiParameters() {
    // Only initialize the parameters if needed
    if (!RECEIVE_WIFI_PARAMS || wifiParameters != null) {
      return;
    }
    System.out.println("Waiting to receive Wi-Fi parameters.");

    // Connect to server and get the data, catching any errors that might occur
    try (WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT)) {
      /*
       * getData() will connect to the server and wait until the user/TA presses the "Start" button in the GUI on their
       * laptop with the data filled in. Once it's waiting, you can kill it by pressing the upper left hand corner
       * button (back/escape) on the EV3. getData() will throw exceptions if it can't connect to the server (e.g. wrong
       * IP address, server not running on laptop, not connected to WiFi router, etc.). It will also throw an exception
       * if it connects but receives corrupted data or a message from the server saying something went wrong. For
       * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot will receive a
       * message saying an invalid team number was specified and getData() will throw an exception letting you know.
       */
      wifiParameters = conn.getData();

      notGotWifi = false;
    } catch (Exception e) {
      LCD.clear();
      System.err.println("Error: " + e.getMessage());
      if(e.getMessage().equals("Bad server status: TEAM NUMBER ERROR")) {
        notGotWifi = true;
      }
    }
  }

  /**
   * Returns the Wi-Fi parameter int value associated with the given key.
   *
   * @param key the Wi-Fi parameter key
   * @return the Wi-Fi parameter int value associated with the given key
   */
  public static int get(String key) {
    if (wifiParameters != null) {
      return ((BigDecimal) wifiParameters.get(key)).intValue();
    } else {
      return 0;
    }
  }

  /**
   * Represents a region on the competition map grid, delimited by its lower-left and upper-right corners (inclusive).
   *
   * @author Younes Boubekeur
   */
  public static class Region {
    /** The lower left corner of the region. */
    public Point ll;

    /** The upper right corner of the region. */
    public Point ur;

    /**
     * Constructs a Region.
     *
     * @param lowerLeft the lower left corner of the region
     * @param upperRight the upper right corner of the region
     */
    public Region(Point lowerLeft, Point upperRight) {
      validateCoordinates(lowerLeft, upperRight);
      ll = lowerLeft;
      ur = upperRight;
    }

    /**
     * Helper constructor to make a Region directly from parameter names.
     *
     * @param llX the Wi-Fi parameter key representing the lower left corner of the region x coordinate
     * @param llY the Wi-Fi parameter key representing the lower left corner of the region y coordinate
     * @param urX the Wi-Fi parameter key representing the upper right corner of the region x coordinate
     * @param urY the Wi-Fi parameter key representing the upper right corner of the region y coordinate
     */
    public Region(String llX, String llY, String urX, String urY) {
      this(new Point(get(llX), get(llY)), new Point(get(urX), get(urY)));
    }

    /**
     * Validates coordinates.
     *
     * @param lowerLeft the lower left corner of the region
     * @param upperRight the upper right corner of the region
     */
    private void validateCoordinates(Point lowerLeft, Point upperRight) {
      if (lowerLeft.x > upperRight.x || lowerLeft.y > upperRight.y) {
        throw new IllegalArgumentException("Upper right cannot be below or to the left of lower left!");
      }
    }

    public String toString() {
      return "[" + ll + ", " + ur + "]";
    }
  }

  /**
   * Represents a coordinate point on the competition map grid.
   *
   * @author Younes Boubekeur
   */
  public static class Point {
    /** The x coordinate. */
    public double x;

    /** The y coordinate. */
    public double y;

    /**
     * Constructs a Point.
     *
     * @param x the x coordinate
     * @param y the y coordinate
     */
    public Point(double x, double y) {
      this.x = x;
      this.y = y;
    }

    public String toString() {
      return "(" + x + ", " + y + ")";
    }

  }

}
