package org.usfirst.frc.team4468.robot.Commands.Routines;

import org.usfirst.frc.team4468.robot.Commands.Drive.StraightDistance;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.AngleRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.HoldingRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeClamp;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeSpeed;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class LineScore extends CommandGroup {

	
    public LineScore() {
        System.out.println("IN INLINE");
        addSequential(new IntakeClamp(Value.kForward));
        addSequential(new StraightDistance(-2.5, 0.5));
        addSequential(new AngleRotate(-140.0, 20));
        addParallel(new HoldingRotate(-140.0));
        addSequential(new IntakeSpeed(-1.0));
        addSequential(new IntakeClamp(Value.kForward));
    }
}
