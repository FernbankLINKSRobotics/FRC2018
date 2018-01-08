package org.usfirst.frc.team4468.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class Constants{
    // Drivetrain
	public static int leftTop  = 0;
	public static int leftMid  = 1;
	public static int leftBot  = 2;
	public static int rightTop = 3;
	public static int rightMid = 4;
	public static int rightBot = 5;

	// IO
	public static int controlJoy = 0;
	public static int rightJoy = 1;
	public static int leftJoy = 2;
	
	//// Encoders
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
}
