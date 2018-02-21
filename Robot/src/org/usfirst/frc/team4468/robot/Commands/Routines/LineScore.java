package org.usfirst.frc.team4468.robot.Commands.Routines;

import org.usfirst.frc.team4468.robot.Commands.Drive.StraightDistance;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.AngleRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.HoldingRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeClamp;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeSpeed;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class LineScore extends CommandGroup {

    public LineScore() {
        System.out.println("IN RUN");
        addSequential(new StraightDistance(-2.75, 0.03));
        addSequential(new AngleRotate(140.0));
        addParallel(new HoldingRotate(140.0));
        addParallel(new IntakeSpeed(-1.0));
        addSequential(new IntakeClamp(Value.kReverse));
    }
}
