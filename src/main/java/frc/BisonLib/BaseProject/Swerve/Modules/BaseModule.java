package frc.BisonLib.BaseProject.Swerve.Modules;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/*
 * This abstract class has all the basic methods a Module should have
 */
public abstract class  BaseModule {

    /*
     * The module index
     * This follows the quadrants in a cartesian coordinate grid
     * 
     * front right - 0
     * front left - 1
     * back left - 2
     * back right - 3
     */
    public int index;

    public BaseModule(int moduleIndex){
        this.index = moduleIndex;
    }

    /**
     * Gets the drive velocity in rotations/sec
     * 
     * @return drive velocity in rotations/sec
     */
    protected abstract double getRawDriveVelocity();

    /**
     * Gets the drive acceleration in rotations/sec^2
     * 
     * @return drive acceleration in rotations/sec^2
     */
    protected abstract double getRawDriveAcceleration();

    /**
     * Gets the drive position in rotaions
     * 
     * @return position in rotations
     */
    protected abstract double getRawDrivePosition();

    protected abstract double getCANCoderRadians();
    public abstract void configDriveMotor();
    public abstract void configTurnMotor();
    public abstract void configCANcoder(double offset);
    public abstract void stop();
    public abstract void setDesiredState(SwerveModuleState state);
    public abstract TalonFX getDriveMotor();
    public abstract TalonFX getTurnMotor();
    public abstract String getModuleType();
    public abstract double getDriveStatorCurrent();
    public abstract void driveWithVoltage(double volts);
    public double getDriveVelocity(){
        return getRawDriveVelocity() / Constants.Swerve.DRIVING_GEAR_RATIO * Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS;
    }
    public double getDriveAcceleration(){
        return getRawDriveAcceleration() / Constants.Swerve.DRIVING_GEAR_RATIO * Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getCANCoderRadians()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getRawDrivePosition() / Constants.Swerve.DRIVING_GEAR_RATIO * Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS, new Rotation2d(getCANCoderRadians()));
    }
}