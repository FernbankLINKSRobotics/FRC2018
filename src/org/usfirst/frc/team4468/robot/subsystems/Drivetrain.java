package org.usfirst.frc.team4468.robot.subsystems;

import org.usfirst.frc.team4468.robot.Constants;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Drivetrain extends Subsystem {

    private SpeedControllerGroup leftMotors = new SpeedControllerGroup(
        new VictorSP(Constants.leftTop),
        new VictorSP(Constants.leftMid),
        new VictorSP(Constants.leftBot)
    );
    
    private SpeedControllerGroup rightMotors = new SpeedControllerGroup(
        new VictorSP(Constants.rightTop),
        new VictorSP(Constants.rightMid),
        new VictorSP(Constants.rightBot)
    );
    
    public Drivetrain() {
        leftMotors.setInverted(true);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void drive(double left, double right) {
        leftMotors .set(left);
        rightMotors.set(right);
    }
    
    public void stop() {
        leftMotors .stopMotor();
        rightMotors.stopMotor();
    }
}

