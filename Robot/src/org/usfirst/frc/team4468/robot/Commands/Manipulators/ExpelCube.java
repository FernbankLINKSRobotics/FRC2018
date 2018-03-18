package org.usfirst.frc.team4468.robot.Commands.Manipulators;

import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ExpelCube extends Command {
    private Intake in = Robot.intake;
    
    private Value clamp;
    
    public ExpelCube(Value v, double speed) {
        requires(in);
        clamp = v;
    }
    
    /* Called repeatedly when this Command is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute(double speed) {
        in.clamp(clamp);
        in.setSpeed(speed);
    }

    /* Make this return true when this Command no longer needs to run execute()
     * @return the command stops when true
     */
    protected boolean isFinished() {
        return !(Robot.oi.ctrl.getRawButton(5) || Robot.oi.ctrl.getRawButton(5));
    }
}
