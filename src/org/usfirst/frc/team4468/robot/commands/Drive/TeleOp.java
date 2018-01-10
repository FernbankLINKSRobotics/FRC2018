package org.usfirst.frc.team4468.robot.commands.Drive;

import org.usfirst.frc.team4468.robot.OI;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TeleOp extends Command {
    
    private Drivetrain dt = Robot.drive;
    private OI oi = Robot.oi;

    public TeleOp() {
        requires(dt);
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        dt.drive(oi.left.getY(), oi.left.getY());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        dt.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        dt.stop();
    }
}
