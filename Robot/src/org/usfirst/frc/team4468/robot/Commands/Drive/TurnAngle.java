package org.usfirst.frc.team4468.robot.Commands.Drive;

import org.usfirst.frc.team4468.robot.Constants;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Util.PID;
import org.usfirst.frc.team4468.robot.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Uses the gyro to turn the robot to a spocifc angle
 */
public class TurnAngle extends Command {
    
    private double theta;
        
    private Drivetrain dt = Robot.drive;
    private PID pid;
        
    public TurnAngle(double t) {
        requires(dt);
        theta = t;
            
        pid = new PID(Constants.angleP, Constants.angleI, Constants.angleD);
        pid.setInputRange(-180, 180);
        pid.setAbsTolerance(1);
        pid.setOutputRange(-1, 1);
        pid.setPoint(theta);
    }
        
    /* Called repeatedly when this Command is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
        dt.drive(pid.calculate(dt.getAngle()), 0);
    }

    /* Make this return true when this Command no longer needs to run execute()
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#isFinished()
     * @return the command stops when true
     */
    protected boolean isFinished() {
        return pid.onTarget(dt.getAngle());
    }

    /* Called once after isFinished returns true
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#end()
     */
    protected void end() {
        dt.stop();
    }

    /* Called when another command which requires one or more of the same
     * subsystems is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#interrupted()
     */
    protected void interrupted() {
        dt.stop();
    }
}
