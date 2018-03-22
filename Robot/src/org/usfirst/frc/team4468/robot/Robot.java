
package org.usfirst.frc.team4468.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4468.robot.Commands.Drive.StraightDistance;
import org.usfirst.frc.team4468.robot.Commands.Drive.TurnAngle;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.AngleRotate;
import org.usfirst.frc.team4468.robot.Commands.Routines.*;
import org.usfirst.frc.team4468.robot.Subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
    public static RotatingLift rotatingLift;
    public static Constants    constants;
    public static Intake       intake;
	public static Drivetrain   drive;
	public static Shifter      shift;
	public static OI           oi;
	public static Run runFunction;
	public static AngleRotate angleRotate;
	
	public static double theta;
	
	private boolean doingSide;
	private boolean doingRun;
	private boolean doingSwitch;
	
	private boolean isRight;
	private boolean isCenter;
	private boolean isLeft;
	//private boolean isSide;
	
	SendableChooser<CommandGroup> autoChooser;
	Command autonomousCommand;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		rotatingLift = new RotatingLift();
		constants    = new Constants();
		intake       = new Intake();
		shift        = new Shifter();
		drive        = new Drivetrain();
		oi           = new OI();
		
		
		/*
		autoChooser = new SendableChooser<CommandGroup>();
		autoChooser.addDefault("PID Tune", new LineScore());
		autoChooser.addObject("Auto Run", new Run());
		autoChooser.addObject("Straight forward switch", new LineScore());
		*/
		
		SmartDashboard.putBoolean("doingLine", doingSwitch);
		SmartDashboard.putBoolean("doingSide", doingSide);
		SmartDashboard.putBoolean("doingRun", doingRun);
		SmartDashboard.putBoolean("isLeft", isLeft);
		SmartDashboard.putBoolean("isRight", isRight);
		SmartDashboard.putBoolean("isCenter", isCenter);
		//SmartDashboard.putData("Auto Chooser", autoChooser);
		UsbCamera cam = CameraServer.getInstance().startAutomaticCapture("Camera", "/dev/video0");
		cam.setVideoMode(VideoMode.PixelFormat.kMJPEG, 256, 144, 30);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		
		Scheduler.getInstance().removeAll();
		
	    drive.encoderReset();
	    drive.gyroReset();
	    if(SmartDashboard.getBoolean("doingRun", false)) {
	    	new Run().start();
	    } else if (SmartDashboard.getBoolean("doingSide", false) 
	    		&& DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R' 
	    		&& SmartDashboard.getBoolean("isRight", false)) {
	    	new RightDeposit().start();
	    } else if (SmartDashboard.getBoolean("doingSide", false) 
	    		&& DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'L' 
	    		&& SmartDashboard.getBoolean("isLeft", false)) {
	    	new LeftDeposit().start();
	    } else if (SmartDashboard.getBoolean("doingSwitch", false) 
	    		&& SmartDashboard.getBoolean("isLeft", false) 
	    		&& DriverStation.getInstance().getGameSpecificMessage().charAt(0)=='L') {
	    	new LineScore().start();
	    } else if (SmartDashboard.getBoolean("doingSwitch", false) 
	    		&& SmartDashboard.getBoolean("isRight", false) 
	    		&& DriverStation.getInstance().getGameSpecificMessage().charAt(0)=='R') {
	    	new LineScore().start();
	    } else if (SmartDashboard.getBoolean("isCenter", false) 
	    		&& DriverStation.getInstance().getGameSpecificMessage().charAt(0)=='L') {
	    	new CenterAuto().start();
	    } else if (SmartDashboard.getBoolean("isCenter", false) 
	    		&& DriverStation.getInstance().getGameSpecificMessage().charAt(0)=='R') {
	    	new CenterAuto().start();
	    } else {
	    	new Run().start();
	    }
		
		//angleRotate = new AngleRotate(-140.0, -.01);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		//SmartDashboard.putNumber("LeftENC" , drive.getLeftDistance());
        //SmartDashboard.putNumber("RightENC", drive.getRightDistance());
        //SmartDashboard.putNumber("Gyro", drive.getAngle());
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		Scheduler.getInstance().removeAll();
		
		drive.encoderReset();
		drive.gyroReset();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		log();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		log();
	
	}
	
	public void log() {
	//	System.out.println("Right Encoder Distance:" + drive.pulsesToDistance(drive.getRightDistance()));
	//	System.out.println("Left Encoer Distance:"   + drive.pulsesToDistance(drive.getLeftDistance()));
	//	System.out.println("Right Encoder Ticks:" + drive.getRightDistance());
      //  System.out.println("Left Encoer Ticks:"   + drive.getLeftDistance());
        //System.out.println("Controller 1" + oi.ctrl.getY(Hand.kLeft));
        //System.out.println("PID Rotate:" + rotatingLift.getAngle());
        //System.out.println("Angle:" + drive.getAngle());
        SmartDashboard.putNumber("LeftENC" , drive.getLeftDistance());
        SmartDashboard.putNumber("RightENC", drive.getRightDistance());
        SmartDashboard.putNumber("Petentiometer", rotatingLift.getAngle());
	}
}
