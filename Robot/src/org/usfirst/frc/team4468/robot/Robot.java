
package org.usfirst.frc.team4468.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4468.robot.Commands.Drive.LeftDistance;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.RotateAngle;
import org.usfirst.frc.team4468.robot.Commands.Routines.Run;
import org.usfirst.frc.team4468.robot.Subsystems.*;
import org.usfirst.frc.team4468.robot.Util.PID;

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
	
	public static double theta;
	
	SendableChooser<CommandGroup> autoChooser;
	Command autonomousCommand;
	Command liftCommand;
	
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
		runFunction  = new Run();
		
		
		autoChooser = new SendableChooser<CommandGroup>();
		autonomousCommand = new Run();
		theta = 90;
		//liftCommand = new RotateAngle();
		//liftCommand.start();
		//autoChooser.addDefault("PID Tune", new Run());
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
		//autonomousCommand = autoChooser.getSelected();
		System.out.println("Starting Auto");
		if (autonomousCommand != null) {
			System.out.println("In If Statement");
			//autonomousCommand.start()
			//runFunction.llama();
		}
		//liftCommand.start();

		//new LeftDistance(2).start();
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		System.out.println("In Periodic");
		log();
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
		//.rotate(oi.ctrl.getY(Hand.kLeft));
		//intake.setSpeed(oi.ctrl.getY(Hand.kRight));
		log();
	
	}
	
	public void log() {
		System.out.println("Right Encoder Distance:" + drive.pulsesToDistance(drive.getRightDistance()));
		System.out.println("Left Encoer Distance:"   + drive.pulsesToDistance(drive.getLeftDistance()));
		System.out.println("Right Encoder Ticks:" + drive.getRightDistance());
        System.out.println("Left Encoer Ticks:"   + drive.getLeftDistance());
        System.out.println("Controller 1" + oi.ctrl.getY(Hand.kLeft));
        System.out.println("PID Rotate:" + rotatingLift.getAngle());
        SmartDashboard.putNumber("LeftENC" , drive.getLeftDistance());
        SmartDashboard.putNumber("RightENC", drive.getRightDistance());
        SmartDashboard.putNumber("Petentiometer", rotatingLift.getAngle());
        double why = 20.0;
        double me = 360.0;
        double hatehate= why/me;
        System.out.println("I can't do math: " + (hatehate));
	}
}
