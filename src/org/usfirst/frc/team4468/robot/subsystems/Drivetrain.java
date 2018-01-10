package org.usfirst.frc.team4468.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team4468.robot.Constants;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import org.usfirst.frc.team4468.robot.commands.Drive.TeleOp;
/**
 *
 */
public class Drivetrain extends Subsystem {
    //// Declarations
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
    
    
    
    //// Class Inits
    // Automatic actions needed for the rest of the computation
    public Drivetrain() {
        leftMotors.setInverted(true);
    }
    
    /* This is saying that when the robot starts up this sub calls
     * the command for general tank drive
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Subsystem#initDefaultCommand()
     */
    public void initDefaultCommand() {
        setDefaultCommand(new TeleOp());
    }
    
    
    
    //// Drive Motor Code
    /* Sets the motors in for a tank drive
     * @param left  the values to the left side of the drive train
     * @param right the right values to the right side
     */
    public void drive(double left, double right) {
        leftMotors .set(left);
        rightMotors.set(right);
    }
    
    // Stops all of the motors
    public void stop() {
        leftMotors .stopMotor();
        rightMotors.stopMotor();
    }
    
    
    
    //// Encoder Getters
    // The average distance between the motors
    public double getDis() {
        return ave(leftEncoder.getDistance(), rightEncoder.getDistance());
    }
    
    // The average velocity between the motors
    public double getVel() {
        return ave(leftEncoder.getRate(), rightEncoder.getRate());
    }
    
    // The average acceleration between the motors
    public double getAcc() {
        double leftVel  = leftEncoder .getRate();
        double rightVel = rightEncoder.getRate();
        
        leftPrevVel  = leftVel;
        rightPrevVel = rightVel;
        
        return ave((leftVel - leftPrevVel), (rightVel- rightPrevVel));
    }
    
    // Returns the average of two values
    private double ave(double left, double right) {
        return (left + right) / 2;
    }
    
    
    //// Resets Sensors
    // Resets the encoders to Zero
    public void encoderReset() {
        leftEncoder .reset();
        rightEncoder.reset();
    }
    
    // resets the gyro back to Zero
    public void gyroReset() {
        gyro.reset();
    }
}

