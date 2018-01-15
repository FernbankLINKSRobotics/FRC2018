package org.usfirst.frc.team4468.robot.commands.Manipulators;

import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeSpeed extends Command {

	private Intake in = Robot.intake;
	
	private double speed = 0;
	
    public IntakeSpeed(double s) {
    		requires(in);
    		speed = s;
    		
    }


    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	in.setSpeed(speed);
    	
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    		in.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		in.stop();
    }
}
