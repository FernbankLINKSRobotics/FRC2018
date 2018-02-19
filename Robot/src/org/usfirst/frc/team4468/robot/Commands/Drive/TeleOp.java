package org.usfirst.frc.team4468.robot.Commands.Drive;

import org.usfirst.frc.team4468.robot.OI;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TeleOp extends Command {
    
    private boolean isArcade;
    
    private Drivetrain dt = Robot.drive;
    private OI oi = Robot.oi;

    public TeleOp(boolean b) {
        requires(dt);
        
        isArcade = b;
    }
    
    /* Called repeatedly when this Command is scheduled to run
     * this drives the tank based on the values of the joysticks
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
        if(isArcade) {
            dt.arcade(oi.drvr.getX(Hand.kLeft), -oi.drvr.getY(Hand.kRight));
        } else {
            dt.tank(oi.drvr.getY(Hand.kLeft), -oi.drvr.getY(Hand.kRight));
        }
    }

    /* It will NEVER end
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#isFinished()
     * @return true when this Command no longer needs to run execute()
     */
    protected boolean isFinished() {
        return false;
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
