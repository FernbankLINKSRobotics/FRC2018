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
        addSequential(new StraightDistance(-2.5));
        addSequential(new AngleRotate(140));
        addParallel(new HoldingRotate(140));
        addParallel(new IntakeSpeed(-1));
        addSequential(new IntakeClamp(Value.kReverse));
    }
}