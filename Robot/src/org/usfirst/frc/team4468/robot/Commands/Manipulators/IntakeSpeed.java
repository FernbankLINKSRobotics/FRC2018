package org.usfirst.frc.team4468.robot.Commands.Manipulators;

import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeSpeed extends Command {
    
    private Intake in = Robot.intake;
    private double speed = 0.0;
    
    public IntakeSpeed(double s) {
        requires(in);
        speed = s;
    }
    
    /* Called repeatedly when this Command is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
    		System.out.println("WE HEREE");
        in.setSpeed(speed);
    }

    /* Make this return true when this Command no longer needs to run execute()
     * @return the command stops when true
     */
    protected boolean isFinished() {
        return false;
    }

    /* Called once after isFinished returns true
     */
    protected void end() {
        in.stop();
    }

    /* Called when another command which requires one or more of the same
     * subsystems is scheduled to run
     */
    protected void interrupted() {
        in.stop();
    }
}
