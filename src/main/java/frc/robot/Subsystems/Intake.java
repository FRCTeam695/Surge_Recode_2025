package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.revrobotics.spark.*;


public class Intake extends SubsystemBase{
    
    private SparkMax intakeMotor;

    private SparkMax indexMotor;
    private SparkClosedLoopController indexerPIDController;
    private RelativeEncoder indexerEncoder;
    public final Trigger beamIsBroken;

    DigitalInput beamBreak;
    boolean noteStatus;

    public Intake(){
        intakeMotor = new SparkMax(51, SparkLowLevel.MotorType.kBrushless);
        intakeMotor.clearFaults();

        indexMotor = new SparkMax(52, SparkLowLevel.MotorType.kBrushless);
        indexerEncoder = indexMotor.getEncoder();
        indexerPIDController = indexMotor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(50);
        config.idleMode(IdleMode.kBrake);
        intakeMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        indexMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        intakeMotor.clearFaults();
        indexMotor.clearFaults();


        beamBreak = new DigitalInput(0);
        noteStatus = false;
        beamIsBroken = new Trigger(this::getBeamBreak);
    }

    public Command setNoteStatus(boolean hasNote){
        return runOnce(()-> noteStatus = hasNote);
    }

    public boolean getBeamBreak(){
        return beamBreak.get();
    }

    public boolean getBeamMade(){
        return !getBeamBreak();
    }

    public boolean getNoteStatus(){
        return noteStatus;
    }

    public Command runIntakeAndIndexerPercent(double percentVBus){
        return run(
            ()-> {
                runIntakeToSpeed(percentVBus);
                indexMotor.set(-percentVBus);
            }
        );
    }

    public void runIntakeToSpeed(double speed){
        intakeMotor.set(-speed);
    }

    public Command runIndexerToSpeed(double speed){
        return run(
            ()-> indexMotor.set(-speed)
        );
    }

    public Command indexerClosedLoopControl(double rotations, double kp){
        return new FunctionalCommand(
            ()-> {
                  indexerEncoder.setPosition(0);
                 },
            ()-> indexerPIDController.setReference(-rotations, SparkMax.ControlType.kPosition),
            interrupted-> {},
            ()-> indexerEncoder.getPosition() <= -rotations,
            this
        );
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Beambreak", getBeamBreak());
    }
}
