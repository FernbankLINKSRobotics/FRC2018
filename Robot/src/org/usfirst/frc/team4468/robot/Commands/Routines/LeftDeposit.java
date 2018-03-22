package org.usfirst.frc.team4468.robot.Commands.Routines;

import org.usfirst.frc.team4468.robot.Commands.Drive.StraightDistance;
import org.usfirst.frc.team4468.robot.Commands.Drive.TurnAngle;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.AngleRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.HoldingRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeClamp;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeSpeed;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class LeftDeposit extends CommandGroup{
	public LeftDeposit() {
		addSequential(new IntakeClamp(Value.kReverse));
		addSequential(new StraightDistance(-3.00, .5));
		addSequential(new TurnAngle(-90, 5));
		addSequential(new StraightDistance(-.5, .1));
		addSequential(new AngleRotate(-140.0, 20));
		addParallel(new HoldingRotate(-140));
	}

}
