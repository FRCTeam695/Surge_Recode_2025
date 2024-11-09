package frc.BisonLib.BaseProject.Controller;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class EnhancedCommandController extends CommandXboxController{
    
    public EnhancedCommandController(int port){
        super(port);
    }


    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                boolean temp = (alliance.get() == DriverStation.Alliance.Red) ? true : false;
                SmartDashboard.putBoolean("Alliance", temp);
                return temp;
              }
        SmartDashboard.putBoolean("Alliance", false);
        return false;
    }


    public ChassisSpeeds getRequestedChassisSpeeds(){

        // +X is forward and +Y is left in wpilib coordinates
        double Xj = getLeftY();
        double Yj = getLeftX();

        // +Z is ccw
        double Zj = -getSquaredRightStick();

        if(!isRedAlliance()){
            Xj *= -1;
            Yj *= -1;
        }
        double db = 0.2;

        Xj = MathUtil.applyDeadband(Xj, db);
        Yj = MathUtil.applyDeadband(Yj, db);
        Zj = MathUtil.applyDeadband(Zj, db);

        //WANTED FIELD RELATIVE VELOCITIES
        Xj *= Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS;
        Yj *= Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS;
        Zj *= Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECOND;

        SmartDashboard.putNumber("Zj", Zj);
        SmartDashboard.putNumber("Xj", Xj);
        SmartDashboard.putNumber("Yj", Yj);

        return new ChassisSpeeds(Xj, Yj, Zj);
    }


    /**
     * Squares the rightX stick values, makes the robot accel. less for smaller joystick inputs
     * 
     * @return the squared right stick values
     */
    public double getSquaredRightStick(){
        double original = super.getRightX();
        if(original > 0) return Math.pow(original, 2);
        return Math.pow(original, 2) * -1;
    }

    /**
     * Rumbles the controller to a given strength
     * 
     * @param strength A Double Supplier that defines the rumbles intensity
     * @return A Command that rumbles to a strength and turns off the rumble when finished
     */
    public Command rumble(DoubleSupplier strength){
        return runEnd
                  (
                    ()-> super.getHID().setRumble(RumbleType.kBothRumble, strength.getAsDouble()),
                    ()-> super.getHID().setRumble(RumbleType.kBothRumble, 0)
                  );
    }
}
