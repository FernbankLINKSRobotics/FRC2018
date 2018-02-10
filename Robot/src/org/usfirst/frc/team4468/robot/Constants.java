package org.usfirst.frc.team4468.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class Constants{
    //// System Units
    // General
    public static double weight = 59; // Placeholder
    // Wheel
    public static double distanceBetweenWheels = 27;
    public static double wheelDiameter = 0.1524;
    public static double wheelCircumference = wheelDiameter * Math.PI;
    // Gears
    public static double highUnstagedGearRatio = 12/125;
    public static double lowUnstagedGearRatio = 6/17;
    public static double stageRatio = 64/20;
    public static double encoderRatio = 1/3;
    public static double maxVelocity = 6;
    // Driver Input
    public static double deadband = 0.1;
    
    ///// Drivetrain
    // Left Side
	public static int leftTop  = 0;
	public static int leftMid  = 1;
	public static int leftBot  = 3;
	public static double leftP = 0;
	public static double leftI = 0;
	public static double leftD = 0;
	// Right Side
	public static int rightTop  = 4;
	public static int rightMid  = 5;
	public static int rightBot  = 6;
	public static double rightP = 0;
	public static double rightI = 0;
	public static double rightD = 0;

	// IO
	public static int driveController = 0;
	public static int operatorController = 1;
	
	//// Encoders
	// Constants
	public static double pulsesPerRev = 128;
	public static double distancePerPulse = 0.000396745; //wheelCircumference / (pulsesPerRev * (stageRatio * encoderRatio));
	// Left Encoder
	public static boolean leftEncInverted = true;
	public static int leftEnc2 = 1;
	public static int leftEnc1 = 0;
	// Right Encoder
    public static boolean rightEncInverted = false;
    public static int rightEnc2 = 3;
    public static int rightEnc1 = 2;
    
    // Gyro
    public static edu.wpi.first.wpilibj.I2C.Port gyroPort = I2C.Port.kOnboard;
    public static double angleP = 0;
    public static double angleI = 0;
    public static double angleD = 0;
    
    // Shifter
    public static int shifterPort1 = 0;
    public static int shifterPort2 = 1;
    
    // Intake
    public static int intakePort1 = 7;
    public static int intakePort2 = 8;
    public static int intakeClampPort1 = 2;
    public static int intakeClampPort2 = 3;
    
    // Lifter
    public static int lifterPort1 = 9;
    public static double lifterP  = 0;
    public static double lifterI  = 0;
    public static double lifterD  = 0;
    
    // Potentiometer
    public static int potPort = 0; // Placeholder
    public static double potRange = 3600; // Placeholder
    public static double potOff = 0; // Placeholder
    
    
    //// System functions
    private static double prevTime = 0;
    public static double dt() {
        double time = Timer.getFPGATimestamp();
        double delta = time - prevTime;
        prevTime = time;
        return delta/1000;
    }
}
