package org.usfirst.frc.team4468.robot.commands.Drive;

import org.usfirst.frc.team4468.robot.Constants;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Util.PID;
import org.usfirst.frc.team4468.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RightDistance extends Command {
    private double distance;
    
    private Drivetrain dt = Robot.drive;
    private PID pid;
        
    public RightDistance(double d) {
        requires(dt);
        distance = d;
            
        pid = new PID(Constants.rightP, Constants.rightI, Constants.rightD);
        pid.setOutputRange(-1, 1);
        pid.setPoint(distance);
    }
        
    /* Called repeatedly when this Command is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
        dt.drive(0, pid.calculate(dt.getRightDistance()));
    }

    /* Make this return true when this Command no longer needs to run execute()
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#isFinished()
     * @return the command stops when true
     */
    protected boolean isFinished() {
        return pid.onTarget(dt.getRightDistance());
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
