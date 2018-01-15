package org.usfirst.frc.team4468.robot.commands.Manipulators;

import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.subsystems.RotatingLift;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Rotate extends Command {

	private double speed = 0;
	private RotatingLift rl = Robot.rotatingLift;
	
    public Rotate(double s) {
        requires(rl);
        speed = s;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		Robot.rotatingLift.rotate(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !Robot.oi.left.getRawButton(2) || !Robot.oi.left.getRawButton(2);
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
