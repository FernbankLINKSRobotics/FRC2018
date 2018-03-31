package org.usfirst.frc.team4468.robot;

import org.usfirst.frc.team4468.robot.Commands.Routines.CenterAuto;
import org.usfirst.frc.team4468.robot.Commands.Routines.GyroTest;
import org.usfirst.frc.team4468.robot.Commands.Routines.LeftDeposit;
import org.usfirst.frc.team4468.robot.Commands.Routines.LineScore;
import org.usfirst.frc.team4468.robot.Commands.Routines.RightDeposit;
import org.usfirst.frc.team4468.robot.Commands.Routines.Run;
import org.usfirst.frc.team4468.robot.Subsystems.Drivetrain;
import org.usfirst.frc.team4468.robot.Subsystems.Intake;
import org.usfirst.frc.team4468.robot.Subsystems.RotatingLift;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	public static RotatingLift rotatingLift;
	public static Constants constants;
	public static Intake intake;
	public static Drivetrain drive;
	public static OI oi;
	
	public static double theta;
	
	// Routine option
	private boolean doingSide;
	private boolean doingRun;
	private boolean doingLine;
	// Robot position
	private boolean isRight;
	private boolean isCenter;
	private boolean isLeft;
	// Testing
	private boolean testingGyro;
	private boolean testingStraight;
	
	@Override 
	public void robotInit() {
		rotatingLift = new RotatingLift();
		constants = new Constants();
		intake = new Intake();
		drive = new Drivetrain();
		oi = new OI();
		
		// The auto routines
		SmartDashboard.putBoolean("doingLine", doingLine);
		SmartDashboard.putBoolean("doingSide", doingSide);
		SmartDashboard.putBoolean("doingRun", doingRun);
		// The positions
		SmartDashboard.putBoolean("isLeft", isLeft);
		SmartDashboard.putBoolean("isRight", isRight);
		SmartDashboard.putBoolean("isCenter", isCenter);
		// Testing
		SmartDashboard.putBoolean("testingGyro", testingGyro);
		SmartDashboard.putBoolean("testingStraight", testingStraight);
		
		UsbCamera cam = CameraServer.getInstance().startAutomaticCapture("Camera", "/dev/video0");
		cam.setVideoMode(VideoMode.PixelFormat.kMJPEG, 265, 144, 30);
	}
	
	@Override
	public void autonomousInit() {
		Scheduler.getInstance().removeAll();
		
		drive.encoderReset();
		drive.gyroReset();
		if(SmartDashboard.getBoolean("doingRun", false)) {
			new Run().start();
		} else if (SmartDashboard.getBoolean("doingSide", false)
					&& SmartDashboard.getBoolean("isLeft", false)
					&& DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'L') {
						new LeftDeposit().start();	
		}else if (SmartDashboard.getBoolean("doingSide", false)
					&& SmartDashboard.getBoolean("isRight", false)
					&& DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R') {
						new RightDeposit().start();	
		}else if (SmartDashboard.getBoolean("doingLine", false)
					&& SmartDashboard.getBoolean("isLeft", false)
					&& DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'L') {
						new LineScore().start();
		} else if (SmartDashboard.getBoolean("doingLine", false)
					&& SmartDashboard.getBoolean("isRight", false)
					&& DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R') {
						new LineScore().start();
		} else if (SmartDashboard.getBoolean("isCenter", false)
					&& DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'L') {
						new CenterAuto().start();
		} else if (SmartDashboard.getBoolean("isCenter", false)
					&& DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R') {
						new CenterAuto().start();
		} else if (SmartDashboard.getBoolean("testingGyro", false)) {
						new GyroTest().start();
		} else if (SmartDashboard.getBoolean("testingStraight",  false)) {
						new Run().start();
		} else {
						new Run().start();
	}
		// angleRotate = new AngleRotate(-140.0, -.01);
}
	
	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		log();
	}
	
	@Override
	public void teleopInit() {
		/* THis makes sure that the autonomous stops running when
		 *  teleop starts running. If you want the autonomous
		 *  to continue until interrupted by another command
		 *  remove this line or comment it out
		 */
		drive.encoderReset();
		drive.gyroReset();
																	
	}
	
	public void teleopPeriodic () {
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
		SmartDashboard.putNumber("LeftENC", drive.getLeftDistance());
		SmartDashboard.putNumber("RightENC", drive.getRightDistance());
		SmartDashboard.putNumber("Petentiometer", rotatingLift.getAngle());
		SmartDashboard.putNumber("TurnAngle", drive.getAngle());
	}
}

