package org.usfirst.frc.team4468.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team4468.robot.Constants;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import org.usfirst.frc.team4468.robot.commands.Drive.Joystick;
/**
 *
 */
public class Drivetrain extends Subsystem {
    // Class Declarations
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
    
    public Encoder leftEncoder = new Encoder(
            Constants.leftEnc1, 
            Constants.leftEnc2, 
            Constants.leftEncInverted, 
            Encoder.EncodingType.k4X
    );
    
    public Encoder rightEncoder = new Encoder(
            Constants.rightEnc1, 
            Constants.rightEnc2, 
            Constants.rightEncInverted,
            Encoder.EncodingType.k4X
    );
    
    public AHRS gyro = new AHRS(Constants.gyroPort);
    
    // Variable Declarations
    private double leftPrevVel = 0;
    private double rightPrevVel = 0;
    
    // Class Inits
    public Drivetrain() {
        leftMotors.setInverted(true);
    }
    
    public void initDefaultCommand() {
        setDefaultCommand(new Joystick());
    }
    
    // Drive Motor Code
    public void drive(double left, double right) {
        leftMotors .set(left);
        rightMotors.set(right);
    }
    
    public void stop() {
        leftMotors .stopMotor();
        rightMotors.stopMotor();
    }
    
    // Encoder Getters
    public double getDis() {
        return ave(leftEncoder.getDistance(), rightEncoder.getDistance());
    }
      
    public double getVel() {
        return ave(leftEncoder.getRate(), rightEncoder.getRate());
    }
    
    public double getAcc() {
        double leftVel  = leftEncoder .getRate();
        double rightVel = rightEncoder.getRate();
        
        leftPrevVel  = leftVel;
        rightPrevVel = rightVel;
        
        return ave((leftVel - leftPrevVel), (rightVel- rightPrevVel));
    }
    
    private double ave(double left, double right) {
        return (left + right) / 2;
    }
    
    // Resets Sensors
    public void encoderReset() {
        leftEncoder.reset();
        rightEncoder.reset();
    }
      
    public void gyroReset() {
        gyro.reset();
    }
}

