package ca.mcgill.ecse211.project;


import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class is used to define static resources in one place for easy access and to avoid cluttering the rest of the
 * codebase. All resources can be imported at once like this:
 * 
 * <p>
 * {@code import static ca.mcgill.ecse211.lab4.Resources.*;}
 */
public class Resources {
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
  public static final double RADIUS = 170;
  
  /**
   * Offset from the wall (cm).
   */
  public static final int BAND_CENTER = 25;

  /**
   * threshold range.
   */
  public static final double THRESHOLD_RANGE = 7.5;

  /**
   * Width of dead band (cm).
   */
  public static final int BAND_WIDTH = 5;

  /**
   * The wheel radius in centimeters.
   */
  public static final double WHEEL_RAD = 2.160;

  /**
   * The robot width in centimeters.
   */
  // public static final double TRACK = 11.45;
//  public static final double TRACK = 9.85;
  

  // wheel end to wheel end
  public static final double TRACK = 13.6;

  // wheel center to wheel center
  //public static final double TRACK2 = 13.6;




  /**
   * The speed at which the robot moves forward in degrees per second.
   */
  public static final int FORWARD_SPEED = 250;
  /**
   * Speed of slower rotating wheel (deg/sec).
   */
  public static final int MOTOR_LOW = 75;


  /**
   * Speed of slower rotating wheel (deg/sec).
   */
  public static final int MOTOR_NORMAL = 140;

  /**
   * Speed of the faster rotating wheel (deg/sec).
   */
  public static final int MOTOR_HIGH = 205;

  /**
   * The speed at which the robot rotates in degrees per second.
   */
  public static final int ROTATE_SPEED = 100;
  
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
  public static final EV3LargeRegulatedMotor usMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  
  
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

}