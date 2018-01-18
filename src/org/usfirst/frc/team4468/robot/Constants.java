package org.usfirst.frc.team4468.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around. NOTE: all of the measurements are in imperial units so inches 
 * and lbs
 */
public class Constants{
    /* We will be running a 6 miniCIM drive on the Vex PRO 2014 Drive in a Day with
     * a Vex PRO 3 CIM ball shifter.
     */
    
    //// System Units
    // General
    public static double weight = 130;
    // Wheel
    public static double distanceBetweenWheels = 27;
    public static double wheelDiameter = 4;
    public static double wheelCircumference = wheelDiameter * Math.PI;
    // Gears
    public static double highWheelGearRatio = 6/25;
    public static double lowWheelGearRatio = 204/3125;
    public static double highUnstagedGearRatio = 12/125;
    public static double lowUnstagedGearRatio = 6/17;
    public static double stageRatio = 34/50;
    public static double encoderRatio = 3;
    
    ///// Drivetrain
    // Left Side
	public static int leftTop  = 0;
	public static int leftMid  = 1;
	public static int leftBot  = 2;
	public static double leftP = 0;
	public static double leftI = 0;
	public static double leftD = 0;
	// Right Side
	public static int rightTop  = 3;
	public static int rightMid  = 4;
	public static int rightBot  = 5;
	public static double rightP = 0;
	public static double rightI = 0;
	public static double rightD = 0;

	// IO
	public static int controlJoy = 2;
	public static int rightJoy = 1;
	public static int leftJoy = 0;
	
	//// Encoders
	// Constants
	public static double pulsesPerRev = 128;
	public static double distancePerPulse = wheelCircumference / (pulsesPerRev * (encoderRatio * stageRatio));
	// Left Encoder
	public static boolean leftEncInverted = true;
	public static int leftEnc2 = 1;
	public static int leftEnc1 = 0;
	// Right Encoder
    public static boolean rightEncInverted = false;
    public static int rightEnc2 = 3;
    public static int rightEnc1 = 2;
    
    // Gyro
    public static Port gyroPort = SerialPort.Port.kUSB1;
    public static double angleP = 0;
    public static double angleI = 0;
    public static double angleD = 0;
    
    // Shifter
    public static int shifterPort1 = 4;
    public static int shifterPort2 = 5;
    
    // Intake
    public static int intakePort1 = 6;
    public static int intakePort2 = 7;
    
    // Lifter
    public static int lifterPort1 = 0;
    public static int lifterPort2 = 1;
    public static double lifterP  = 0;
    public static double lifterI  = 0;
    public static double lifterD  = 0;
    
    // Potentiometer
    public static int potPort = 0; // Placeholder
    public static double potRange = 312; // Placeholder
    public static double potOff = 0; // Placeholder
}
