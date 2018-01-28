package org.usfirst.frc.team4468.robot.commands.Drive;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.subsystems.Shifter;

public class Shift extends Command {

	public Value target;
	public Shifter sf = Robot.shift;
	
	public Shift(Value i) {
		requires(sf);
		target = i;
	}

	/* Called repeatedly when this Command is scheduled to run
	 * this toggles the shifter from high to low
	 * (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.command.Command#execute()
	 */
	@Override
	protected void execute() {
		if(target != sf.getState()) {
		    sf.set(target);
		}
	}

	/* This stops when the state of the shifter changes either from low to high
	 * or from high to low
	 * (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.command.Command#isFinished()
	 * @return true when this Command no longer needs to run execute()
	 */
	@Override
	protected boolean isFinished() {
		return target == sf.getState(); // Runs until interrupted
	}
}