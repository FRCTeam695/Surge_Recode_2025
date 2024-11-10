package frc.robot.Subsystems;



import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Arm extends SubsystemBase {

  private SparkMax m_pitchControlMotor;
  private Encoder shooter_axle;
  private DutyCycleEncoder abs_encoder;
  private DigitalInput di;

  private ProfiledPIDController m_controller;
  private ArmFeedforward m_feedForward;
  private LinearFilter absolute_moving_avg;
  private double latest_abs_moving_avg;
  private double relative_enc_offset;

  private double setpoint_offset;


  private double prevGoalVelocity;
  private double goal;

  public final Trigger atPositionSetpoint = new Trigger(()-> (atGoal() && true));
  public Arm() {
    setpoint_offset = 0;
    relative_enc_offset = 90-78.2666;
    absolute_moving_avg = LinearFilter.movingAverage(100);
    latest_abs_moving_avg = 0;

    di = new DigitalInput(8);
    abs_encoder = new DutyCycleEncoder(di);
    m_pitchControlMotor = new SparkMax(54, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    SparkMaxConfig sparkConfig = new SparkMaxConfig();
        sparkConfig.smartCurrentLimit(20);
    m_pitchControlMotor.configure(sparkConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    m_pitchControlMotor.clearFaults();


    shooter_axle = new Encoder(7, 6);
    shooter_axle.setDistancePerPulse(1/2048. * 360);

    m_controller = 
            new ProfiledPIDController(
            2.9, //1.8 //3
            0.0,
            0.01,
            new TrapezoidProfile.Constraints(3., 9.));
        // new ProfiledPIDController(
        //     1.5,
        //     0, 
        //     0, 
        //     new TrapezoidProfile.Constraints(3., 10.));
    
    m_feedForward = 
            new ArmFeedforward(
            0.0, //0.025
            0.05, //0.065
            0.2, //0.2 //0.265
            0.0); //0.001
        // new ArmFeedforward(
        //     0, 
        //     0.05, 
        //     0.2, 
        //     0.01);

    prevGoalVelocity = 0;
    goal = 0;
  }

  /*
  public Command goToAngle(double goal){
    
    return run(()-> reachGoal(goal));
  }
  */

  public void setGoal(double newGoal){
    goal = newGoal;
  }

  private double getPosition(){
    return Math.toRadians(shooter_axle.getDistance() + relative_enc_offset);
  }

  private double getAbsolutePosition(){
    return abs_encoder.get() * -360  + 310 + 90 - 79.62;
  }


  public boolean atGoal(){
    return Math.abs(getPosition() - goal) <= 0.05;
  }

  public void resetStateToPresent()
  {
    double pos = getPosition();
    goal = pos;
    m_controller.reset(pos);
  }
    

  private void reachGoal(double goal){
    m_controller.setGoal(goal + Math.toRadians(setpoint_offset));

    // With the setpoint value we run PID control like normal
    // Outputs are negated because of encoder position
    double pidOutput = -1 * m_controller.calculate(getPosition());

    double velocity = m_controller.getSetpoint().velocity;
    double acceleration = (velocity - prevGoalVelocity) / m_controller.getPeriod();

    double feedforwardOutput = -1 * m_feedForward.calculate(getPosition(), velocity, acceleration);

    SmartDashboard.putNumber("PID OUTPUT", pidOutput);
    SmartDashboard.putNumber("FEEDFORWARD OUTPUT", feedforwardOutput);
    // SmartDashboard.putNumber("SETPOINT", m_controller.getSetpoint().position);
    // SmartDashboard.putNumber("GOAL", m_controller.getGoal().position);
    //if(pitch_encoder.getPosition() >= -87 || pidOutput >= 0){
    m_pitchControlMotor.set(pidOutput + feedforwardOutput);
    prevGoalVelocity = velocity;
    // SmartDashboard.putNumber("ARM MOTOR OUTPUT", pidOutput + feedforwardOutput);
  }

  public Command goToPositionNoEnd(DoubleSupplier position) {
    return new FunctionalCommand(
      // We have to reset state to present or else the PID controller 
      // doesn't always have an accurate reading of where it is

      // ** INIT **
      ()-> resetStateToPresent(),

      // ** EXECUTE **
      ()-> setGoal(position.getAsDouble()),

      // ** ON INTERRUPTED **
      interrupted->{},

      // ** END CONDITION **
      ()-> false,

      // ** REQUIREMENTS **
      this)

      // withName is just for named commands
      .withName("Shoot Position");
  }

  public Command goToPosition(DoubleSupplier position) {
    return new FunctionalCommand(
      // We have to reset state to present or else the PID controller 
      // doesn't always have an accurate reading of where it is

      // ** INIT **
      ()-> resetStateToPresent(),

      // ** EXECUTE **
      ()-> setGoal(position.getAsDouble()),

      // ** ON INTERRUPTED **
      interrupted->{},

      // ** END CONDITION **
      this::atGoal,

      // ** REQUIREMENTS **
      this)

      // withName is just for named commands
      .withName("Shoot Position");
  }

  public Command homeArm(){
    return runOnce(()->{
      relative_enc_offset += (latest_abs_moving_avg-Math.toDegrees(getPosition()));
      resetStateToPresent();
    }).ignoringDisable(true);
  }

  public void setRelEnc(){
    relative_enc_offset += (latest_abs_moving_avg-Math.toDegrees(getPosition()));
    resetStateToPresent();
  }

  // public double getArmSpeed(){
  //   return Math.abs(sparkEncoder.getVelocity());
  // }
  public void periodic(){
    double relPos = getPosition();
    SmartDashboard.putNumber("Arm Setpoint Offset", setpoint_offset);
    setpoint_offset = SmartDashboard.getNumber("Arm Setpoint Offset", 0);

    double latest_abs_pos = getAbsolutePosition();
    latest_abs_moving_avg = absolute_moving_avg.calculate(latest_abs_pos);

    //SmartDashboard.putNumber("Arm Velocity", getArmSpeed());
    SmartDashboard.putNumber("Relative Encoder Offset", relative_enc_offset);
    SmartDashboard.putNumber("ARM ENCODER POSITION", Math.toDegrees(relPos));
    SmartDashboard.putNumber("Latest Arm Absolute Position", latest_abs_pos);
    SmartDashboard.putNumber("Arm Absolute Position Moving Average", latest_abs_moving_avg);
    SmartDashboard.putBoolean("Arm at Setpoint", atPositionSetpoint.getAsBoolean());

    SmartDashboard.putNumber("Arm Goal", Math.toDegrees(goal));
    SmartDashboard.putBoolean("AT GOAL", atGoal());
    // SmartDashboard.putNumber("ARM ENCODER RAW", m_encoder.getAbsolutePosition());

    if(goal == 0){
      goal = relPos;
    }
    reachGoal(goal);
  }
}
