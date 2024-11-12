package frc.BisonLib.BaseProject.Swerve.Modules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
    private final PIDController turnFeedback;

    // private final SimpleMotorFeedforward driveFf;
    // private final PIDController driveController;

    /*
     * The type of module that it is
     */
    public final String kModuleType = "TalonFXModule";

    private double rot_sample;

    public TalonFXModule(int driveMotorId, int turnMotorId, double absoluteEncoderOffset, int TurnCANCoderId, int moduleIndex){
        super(moduleIndex);

        driveMotor = new TalonFX(driveMotorId, "drivetrain");
        turnMotor = new TalonFX(turnMotorId, "drivetrain");
        absoluteEncoder = new CANcoder(TurnCANCoderId, "drivetrain");

        configDriveMotor();
        configTurnMotor();
        configCANcoder(absoluteEncoderOffset);

        //Creates the PID controller for turning
        //turningPidController = new PIDController(0.015, 0.0, 0.0);
        turnFeedback = new PIDController(Constants.Swerve.MODULE_KP, 0.0, 0);
        turnFeedback.enableContinuousInput(-180, 180); //Tells the PID controller that 180 and -180 are at the same place

        // driveFf = new SimpleMotorFeedforward(0.011, 0.2);
        // driveController = new PIDController(0.1, 0, 0);
        rot_sample = 0;
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

        driveMotor.getConfigurator().apply(config);
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

    protected double getRawDriveAcceleration(){
        return driveMotor.getAcceleration().getValueAsDouble();
    }

    protected double getRawDrivePosition(){
        return driveMotor.getPosition().getValueAsDouble();
    }

    protected double getCANCoderRadians(){
        double angleRad = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        // SmartDashboard.putNumber("Module " + (this.index + 1) +  " Angle Radians", angleRad);
        // SmartDashboard.putNumber("Module " + (this.index + 1) +  " Angle Degrees", Math.toDegrees(angleRad));
        return angleRad;
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
    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, new Rotation2d(getCANCoderRadians()));
        
        double setpoint = state.angle.getDegrees();
        double turnMotorOutput;

        if(Constants.Swerve.DRIVE_MOTOR_INVERTED){
            //Multiply by -1 above because the falcon is upside down on MK4i's
            turnMotorOutput = -1 * MathUtil.clamp(turnFeedback.calculate(Math.toDegrees(getCANCoderRadians()), setpoint), -1, 1);
        }
        else{
            turnMotorOutput =  MathUtil.clamp(turnFeedback.calculate(Math.toDegrees(getCANCoderRadians()), setpoint), -1, 1);
        }

        //double currentVelocity = getDriveVelocity();

        // if(DriverStation.isAutonomous())
        //     driveMotor.set(driveController.calculate(currentVelocity, state.speedMetersPerSecond) + driveFf.calculate(state.speedMetersPerSecond));
        // else{
        driveMotor.set(Math.cos(Math.toRadians(turnFeedback.getError())) * state.speedMetersPerSecond/Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS);
        //}

        turnMotor.set(turnMotorOutput);

        SmartDashboard.putNumber("Module " + (this.index+1) + "PID output", turnMotorOutput);
        SmartDashboard.putNumber("Module " + (this.index+1) + " Desired Velocity", state.speedMetersPerSecond);
        SmartDashboard.putNumber("Module " + (this.index+1) + " Rotation Setpoint Rad", state.angle.getRadians());
    }
}