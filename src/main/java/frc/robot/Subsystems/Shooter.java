package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Shooter extends SubsystemBase {

  private final SparkFlex shooterNeo1 = new SparkFlex(50, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex shooterNeo2 = new SparkFlex(53, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

  // private final CANSparkFlex armNeo = new CANSparkFlex(0,
  // MotorType.kBrushless);

  private final RelativeEncoder shooterNeoEncoder1 = shooterNeo1.getEncoder();
  private final RelativeEncoder shooterNeoEncoder2 = shooterNeo2.getEncoder();

  private final SparkClosedLoopController shooterNeo1PID = shooterNeo1.getClosedLoopController();
  private final SparkClosedLoopController shooterNeo2PID = shooterNeo2.getClosedLoopController();

  private String scoringStatus;

  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, setPointRPM;

  public final Trigger atVelocitySetpoint = new Trigger(()-> shooterIsUpToSpeed());
  public Trigger readyToIntake;

  /** Creates a new ExampleSubsystem. */
  public Shooter() {
    shooterNeo1.clearFaults();
    shooterNeo2.clearFaults();

    shooterNeoEncoder1.setPosition(0);
    shooterNeoEncoder2.setPosition(0);


    // PID coefficients
    kP = 0.000150 * 1.5;// * 2; **times two for amp
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.00016; //0.000155
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;
    setPointRPM = 0;

    SparkFlexConfig config = new SparkFlexConfig();
    ClosedLoopConfig pidConfig = new ClosedLoopConfig();
    pidConfig.p(kP);
    pidConfig.i(kI);
    pidConfig.d(kD);
    pidConfig.iZone(kIz);
    pidConfig.velocityFF(kFF);
    pidConfig.maxOutput(kMaxOutput);
    pidConfig.minOutput(kMinOutput);

    config.voltageCompensation(11.5);
    config.idleMode(IdleMode.kBrake);
    config.apply(pidConfig);


    shooterNeo1.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    shooterNeo2.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    scoringStatus = "intake";

    // // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
    // SmartDashboard.putNumber("Set Rotations", 0);

    readyToIntake = new Trigger(()-> scoringStatus.equals("intake"));
  }

  // Used in order to know what speed to shoot at, i.e. amp vs speaker
  public Command setScoringStatus(String newStatus){
    return runOnce(
    ()-> {scoringStatus = newStatus;}
    );
  }

  // Returns the scoring status
  public String getScoringStatus(){
    return scoringStatus;
  }

  /**
   * setSpeedCommand
   *
   * @return a command
   */
  public void setPercentVBus(DoubleSupplier percentVBus) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    // double voltage2 = voltage.getAsDouble();
    shooterNeo1.set(-percentVBus.getAsDouble());
    shooterNeo2.set(percentVBus.getAsDouble());

  }

  public boolean currentAboveTwentyAmps(){
    return (shooterNeo1.getOutputCurrent() > 20);// || (shooterNeo2.getOutputCurrent() > 20);
  }

  public boolean currentBelowTwentyAmps(){
    return !currentAboveTwentyAmps();
  }

  // returns true id the shooter is up to speed
  public boolean shooterIsUpToSpeed(){

    double deadband = setPointRPM * (75/2222.0); //unit is RPM
    if(Math.abs(setPointRPM-Math.abs(shooterNeoEncoder2.getVelocity())) <= deadband 
    && Math.abs(setPointRPM-Math.abs(shooterNeoEncoder1.getVelocity())) <= deadband)
    {
      return true;
    }
    return false;
  }

  // returns true if the shooter is not up to speed
  public boolean shooterIsNotUpToSpeed(){
    return !shooterIsUpToSpeed();
  }

  // checks if the shooter is running or not
  public boolean isRunning(){
    return setPointRPM > 5;
  }


  public Command runVelocity(DoubleSupplier velocity) {
    return new FunctionalCommand(
        ()-> {
          // SmartDashboard.putNumber("Commanded Velocity", velocity.getAsDouble());
          // shooterNeo1PID.setIAccum(0);
          // shooterNeo2PID.setIAccum(0);
          // shooterNeo1PID.
        },
        () -> {
          setPointRPM = velocity.getAsDouble() * maxRPM;
          shooterNeo1PID.setReference(setPointRPM, SparkFlex.ControlType.kVelocity);
          shooterNeo2PID.setReference(-setPointRPM, SparkFlex.ControlType.kVelocity);

          //SmartDashboard.putNumber("SetPoint", setPointRPM);
        },
        interrupted-> {
        setPointRPM = 0;
          shooterNeo1PID.setReference(setPointRPM, SparkFlex.ControlType.kVelocity);
          shooterNeo2PID.setReference(-setPointRPM, SparkFlex.ControlType.kVelocity);

          //SmartDashboard.putNumber("SetPoint", setPointRPM);
        },
        ()-> (false),
        this
        
        );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Position 2", shooterNeoEncoder2.getPosition());
    //SmartDashboard.putNumber("Position 1", shooterNeoEncoder1.getPosition());
    // SmartDashboard.putBoolean("Shooter at Setpoint", atVelocitySetpoint.getAsBoolean());
    // SmartDashboard.putBoolean("Shooter Up To Speed", shooterIsUpToSpeed());
    // SmartDashboard.putNumber("Velocity 1", shooterNeoEncoder1.getVelocity());
    // SmartDashboard.putNumber("Velocity 2", shooterNeoEncoder2.getVelocity());
    // SmartDashboard.putBoolean("Ready To Intake", readyToIntake.getAsBoolean());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}