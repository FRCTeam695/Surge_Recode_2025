package frc.BisonLib.BaseProject.Swerve.Modules;

//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import frc.robot.Constants;

public class TalonFXModule extends BaseModule{
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    /*
     * This encoder tells the wheel where it is
     * does not get reset on power cycle, this is what makes it an absolute encoder!
     */
    private final CANcoder absoluteEncoder;

    /*
     * This PID controller is responsible for turning the wheel
     */
    //private final PIDController turnFeedback;

    // private final SimpleMotorFeedforward driveFf;
    // private final PIDController driveController;

    private final PositionVoltage rotationSetter = new PositionVoltage(0.0);
    private SwerveModulePosition latestPosition = new SwerveModulePosition();

    /*
     * The type of module that it is
     */
    public final String kModuleType = "TalonFXModule";

    private double rot_sample;
    private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();

    public TalonFXModule(int driveMotorId, int turnMotorId, double absoluteEncoderOffset, int TurnCANCoderId, int moduleIndex){
        super(moduleIndex);

        driveMotor = new TalonFX(driveMotorId, "drivetrain");
        turnMotor = new TalonFX(turnMotorId, "drivetrain");
        absoluteEncoder = new CANcoder(TurnCANCoderId, "drivetrain");

        configCANcoder(absoluteEncoderOffset);
        configDriveMotor();
        configTurnMotor();

        //Creates the PID controller for turning
        //turningPidController = new PIDController(0.015, 0.0, 0.0);
        //turnFeedback = new PIDController(Constants.Swerve.WHEEL_KP, 0.0, 0);
        //turnFeedback.enableContinuousInput(-180, 180); //Tells the PID controller that 180 and -180 are at the same place

        // driveFf = new SimpleMotorFeedforward(0.011, 0.2);
        // driveController = new PIDController(0.1, 0, 0);
        rot_sample = 0;

        driveMotor.getPosition().setUpdateFrequency(Constants.Swerve.ODOMETRY_UPDATE_RATE_HZ);
        driveMotor.getVelocity().setUpdateFrequency(Constants.Swerve.ODOMETRY_UPDATE_RATE_HZ);
        absoluteEncoder.getAbsolutePosition().setUpdateFrequency(Constants.Swerve.ODOMETRY_UPDATE_RATE_HZ);
    }

    
    /**
     * Configures the drive motor
     */
    public void configDriveMotor(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = Constants.Swerve.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Audio.AllowMusicDurDisable = true;

        if(Constants.Swerve.MODULE_IS_INVERTED){
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        else{
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        driveMotor.getConfigurator().apply(config);
        driveMotor.setPosition(0.0);
    }


    /**
     * Configures the turn motor
     */
    public void configTurnMotor(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Audio.AllowMusicDurDisable = true;

        config.Feedback.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.RotorToSensorRatio = Constants.Swerve.TURNING_GEAR_RATIO;

        if(Constants.Swerve.MODULE_IS_INVERTED){
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        else{
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        config.Slot0.kP = Constants.Swerve.WHEEL_KP;
        config.Slot0.kD = Constants.Swerve.WHEEL_KD;
        config.Slot0.kS = Constants.Swerve.WHEEL_KS;

        turnMotor.getConfigurator().apply(config);
    }


    /**
     * Configures the CANcoder,  this involves giving it an offset
     * 
     * @param offset the offset to give the CANcoder
     */
    public void configCANcoder(double offset){
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cancoderConfigs.MagnetSensor.MagnetOffset = offset;
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        absoluteEncoder.getConfigurator().apply(cancoderConfigs);
    }

    public TalonFX getDriveMotor(){return driveMotor;}
    public TalonFX getTurnMotor(){return turnMotor;}
    public String getModuleType(){return kModuleType;}

    public double getDriveStatorCurrent(){
        return driveMotor.getStatorCurrent().getValueAsDouble();
    }
    
    public void driveWithVoltage(double volts){
        driveMotor.setVoltage(volts);
        SmartDashboard.putNumber("Swerve/Module " + (this.index + 1) + "/Supply Voltage Draw", driveMotor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Module " + (this.index + 1) + "/Voltage Draw", driveMotor.getMotorVoltage().getValueAsDouble());
    }

    public double start_rotation_sample(){
        rot_sample = getRawDrivePosition();
        return rot_sample;
    }

    public double get_change_in_rotation(){
        return Math.abs(getRawDrivePosition() - rot_sample);
    }

    protected double getRawDriveVelocity(){
        return driveMotor.getVelocity().getValueAsDouble();
    }

    protected double getRawDrivePosition(){
        return driveMotor.getPosition().getValueAsDouble();
    }

    protected double getRawDriveAcceleration(){
        return driveMotor.getAcceleration().getValueAsDouble();
    }


    protected double getCANCoderRadians(){
        double angleRad = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        // SmartDashboard.putNumber("Module " + (this.index + 1) +  " Angle Radians", angleRad);
        // SmartDashboard.putNumber("Module " + (this.index + 1) +  " Angle Degrees", Math.toDegrees(angleRad));
        return angleRad;
        //return 0;
    }


    /*
     * Stops the drive and turn motors, drives them to zero velocity
     */
    public void stop(){
        driveMotor.set(0.0);
        turnMotor.set(0.0);
    }


    /**
     * Sets the module to a given state
     * 
     * 2DO - Use feedforward on drive motor
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d latestAngle;

        // this way drive and odometry cant acess the angle in latestPosition at the same time
        lock.readLock().lock();
        try{
            latestAngle = latestPosition.angle;
        }finally{
            lock.readLock().unlock();
        }
        
        var delta = desiredState.angle.minus(latestAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
          desiredState = new SwerveModuleState(
              -desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.kPi));
        } else {
          desiredState = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
        
        driveMotor.set(Math.cos(Math.abs(desiredState.angle.getRadians() -  latestAngle.getRadians())) * desiredState.speedMetersPerSecond/Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS);
        turnMotor.setControl(
            rotationSetter.withPosition(desiredState.angle.getRotations())
        );

        /*
        SmartDashboard.putNumber("Module " + (this.index+1) + " Desired Velocity", desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Module " + (this.index+1) + " Rotation Setpoint Rad", desiredState.angle.getRadians());
        SmartDashboard.putNumber("Module " + (this.index+1) + " Cosine Error", Math.cos(Math.abs(desiredState.angle.getRadians() -  latestAngle.getRadians())));
        SmartDashboard.putNumber("Module " + (this.index+1) + " Latest Angle", latestAngle.getRadians());
        */
    }

    @Override
    public SwerveModulePosition getPosition(){
        latestPosition.distanceMeters = getRawDrivePosition() / Constants.Swerve.DRIVING_GEAR_RATIO * Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS;

        // this way drive and odometry cant acess the angle in latestPosition at the same time
        lock.writeLock().lock();
        try{
            latestPosition.angle = new Rotation2d(getCANCoderRadians());
        }finally{
            lock.writeLock().unlock();
        }
        return latestPosition;
    }

    @Override
    public SwerveModuleState getState(){
        Rotation2d angle;
        lock.readLock().lock();
        try {
            angle = latestPosition.angle;
        }finally{
            lock.readLock().unlock();
        }
        return new SwerveModuleState(getDriveVelocity(), angle);
    }

}