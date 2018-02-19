package org.usfirst.frc.team4468.robot.Commands.Routines;

import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Commands.Drive.LeftDistance;
import org.usfirst.frc.team4468.robot.Commands.Drive.RightDistance;
import org.usfirst.frc.team4468.robot.Commands.Drive.StraightDistance;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeClamp;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeSpeed;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.RotateAngle;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Run extends CommandGroup {
	//public static LeftDistance leftDistance = new LeftDistance(2);
	//public static RightDistance rightDistance = new RightDistance(2);

    public Run() {
        System.out.println("IN RUN");
        addSequential(new StraightDistance(-6000));
        addSequential(new RotateAngle(140));
        addSequential(new IntakeClamp(Value.kReverse));
        addSequential(new IntakeSpeed(-1));
    }
}
