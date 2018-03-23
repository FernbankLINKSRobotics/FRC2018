package org.usfirst.frc.team4468.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
	public static double angleOffset = 1082.0;
    public static double weight = 59.0; // Placeholder
    // Wheel
    public static double distanceBetweenWheels = 27.0;
    public static double wheelDiameter = 0.1524;
    public static double wheelCircumference = wheelDiameter * Math.PI;
    // Gears
    public static double highUnstagedGearRatio = 12.0/125.0;
    public static double lowUnstagedGearRatio = 6.0/17.0;
    public static double stageRatio = 64.0/20.0;
    public static double encoderRatio = 1.0/3.0;
    public static double maxVelocity = 6.0;
    // Driver Input
    public static double deadband = 0.1;
    public static double turnMultiplier = 0.9;
    
    ///// Drivetrain
    // Left Side
	public static int leftPair = 9;
	public static int leftBot  = 8;
	public static double leftP = 9.999999;
	public static double leftI = 0.0;
	public static double leftD = 0.0;
	// Right Side
	public static int rightPair = 7;
	public static int rightBot  = 6;
	public static double rightP = 9.999999;
	public static double rightI = 0.0;
	public static double rightD = 0.0;
	// Both Sides
	public static double lineP = 45.0;
	public static double lineI = 0.01;
	public static double lineD = 0.01;
	

	// IO
	public static int driveController = 0;
	public static int operatorController = 1;
	
	//// Encoders
	// Constants
	public static double pulsesPerRev = 128.0;
	public static double distancePerPulse = 0.000396745; //wheelCircumference / (pulsesPerRev * (stageRatio * encoderRatio));
	// Left Encoder
	public static boolean leftEncInverted = true;
	public static int leftEnc2 = 3;
	public static int leftEnc1 = 2;
	// Right Encoder
    public static boolean rightEncInverted = false;
    public static int rightEnc2 = 1;
    public static int rightEnc1 = 0;
    
    // Gyro
    public static edu.wpi.first.wpilibj.I2C.Port gyroPort = I2C.Port.kOnboard;
    public static double angleP = 0.8;
    public static double angleI = 0.01;
    public static double angleD = 0.012;
    
    // Shifter
    public static int shifterPort1 = 0;
    public static int shifterPort2 = 4;
    
    // Intake
    public static int intakePort1 = 5;
    public static int intakePort2 = 4;
    public static int intakeClampPort1 = 5;
    public static int intakeClampPort2 = 3;
    public static int photoGatePort = 5;
    
    // Lifter
    public static int lifterPort1 = 3;
    public static double lifterP  = 5.0;
    public static double lifterI  = 0.0;
    public static double lifterD  = .12;
    public static int BreakClampPort1 = 1;
    public static int BreakClampPort2 = 2;
    
    // Potentiometer
    public static int potPort = 3; // Placeholder
    public static double potRange = 1125.0; // Placeholder
    public static double potOff = 0.0; // Placeholder
    
    
    //// System functions
    private static double prevTime = 0;
    public static double dt() {
        double time = Timer.getFPGATimestamp();
        double delta = time - prevTime;
        prevTime = time;
        return delta/1000.0;
    }
    //public static PTune = SmartDashboard.getNumber("PID", Constants.angleP);
}
