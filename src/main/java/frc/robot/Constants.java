package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.BisonLib.BaseProject.Swerve.SwerveConfig;

public class Constants {
    public static final class Swerve {
        /*
        stores the robot specific constants {
            frontRightOffset [0],
            frontLeftOffset [1],
            backLeftOffset [2],
            backRightOffset [3],
            drivingGearRatio [4],
            maxSpeedFeetPerSec [5],
            maxAngularSpeedFeetPerSec [6],
            wheelCircumferenceInches [7],
            turningKPval [8],
            profiledKPval [9],
            //maxVelocityMetersPerSec [10],   The number at this index isn't used, 
                                                it gets recalculated, 
                                                i just left this here cus its too hard to change, have to change a lot of code
            wheelbaseInches [11],
            trackwidthInches [12],
            maxAngularAccelerationRadiansPerSecond [13],
            turningGearRatio [14], 
            isMK4i  (1 means it is MK4i, 0 means its not) [15]
            rotationToAngleKPval [16]
        }
         */

         /* 2DO - Make this another constants class instead of an array */
        public static final double[] GOLDMODULE_CONSTANTS = { 338, 107, 311, 29, 5.70, 15.82, 15.82, 4 * Math.PI, 0.015,
                1, 6.35, 21.5, 24.5, Math.PI, 150.0 / 7, 1 , 0};
        public static final double[] QB_CONSTANTS = { 351, 229, 169, 207, 8.14, 10.9, 10.9, 4 * Math.PI, 0.015, 
                1, 6.92, 19, 19, Math.PI, 150.0 / 7, 1 , 0};
        public static final double[] LITEBOT_CONSTANTS = { 194, -5, 2, 268, 6.12, 14.73, 14.73, 4 * Math.PI, 0.007, 
                1, 4.49, 21, 24, Math.PI, 12.8, 0 , 0};
        public static final double[] PRODUCTION_2024 = { (93.42 + 180), 273.5, (180 + 84.11), (180 + 69), 6.12, 14.73, 14.73, 3.8 * Math.PI, 0.01,
                1, 4.49, 22.5, 22.5, Math.PI, 12.8, 0, 0.05};   

        public static final SwerveConfig surgeConfig = new SwerveConfig(-0.007080 + 0.5, -0.009277 + 0.5, -0.484375, -0.441895, 
                                                6.12, Units.metersToFeet(5.3), 3.8 * Math.PI,40, 
                                    5, 22.5, 22.5, 12.8, 
                                    false, 0.006, 6, 0, 110, 40, 0, 0, 10);
        
        public static final SwerveConfig goldmoduleConfig = 
                new SwerveConfig(0.317, 0.459+0.5, -0.112+0.5, 0.163,
                                    //new SwerveConfig(0.317, 0, 0, 0,
                        5.7, 15.82, 4 * Math.PI, 0.015, 1, 22.25, 24.75, 150.0 / 7, true, 0.006, 1, 0, 90, 40, 0, 0, 0);
        // public static final Map<String, double[]> ROBOT_MAP = new HashMap<String, double[]>() {
        //     {
        //         put("GOLDMODULE", GOLDMODULE_CONSTANTS);
        //         put("QB", QB_CONSTANTS);
        //         put("LITEBOT", LITEBOT_CONSTANTS);
        //         put("PRODUCTION_2024", PRODUCTION_2024);
        //     }
        // };

        public static final Map<String, SwerveConfig> ROBOT_MAP = new HashMap<String, SwerveConfig>() {
            {
                put("Surge", surgeConfig);
                put("Goldmodule", goldmoduleConfig);
            }
        };
        

        // CHOOSE WHICH ROBOT YOU'RE USING
        public static final SwerveConfig CHOSEN_CONSTANTS = ROBOT_MAP.get("Surge");

        // miscellaneous constants
        public static final double MAX_SPEED_METERS_PER_SECONDS = CHOSEN_CONSTANTS.maxSpeedMetersPerSec;
        public static final double MAX_ANGULAR_SPEED_RAD_PER_SECOND = CHOSEN_CONSTANTS.maxAngularSpeedRadPerSec;
        public static final double TURNING_GEAR_RATIO = CHOSEN_CONSTANTS.turningGearRatio;
        public static final double DRIVING_GEAR_RATIO = CHOSEN_CONSTANTS.drivingGearRatio;
        public static final double WHEEL_CIRCUMFERENCE_METERS = CHOSEN_CONSTANTS.wheelCircumferenceMeters;
        public static final double WHEEL_KP = CHOSEN_CONSTANTS.wheelKP;
        public static final double WHEEL_KS = CHOSEN_CONSTANTS.wheelKS;
        public static final double WHEEL_KD = CHOSEN_CONSTANTS.wheelKD;
        public static final double ROBOT_ROTATION_KP = CHOSEN_CONSTANTS.rotationOverrideKP;
        public static final double PATHPLANNER_OMEGA_KP = CHOSEN_CONSTANTS.pathplannerOmegaKP;
        public static final double PATHPLANNER_TRANSLATION_KP = CHOSEN_CONSTANTS.pathplannerTranslationKP;
        public static final double MAX_WHEEL_ROTATIONAL_SPEED = CHOSEN_CONSTANTS.maxWheelRotationalSpeed;

        public static final double MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED = CHOSEN_CONSTANTS.maxAngularAccelerationRadPerSecondSquared;
        public static final double ROBOT_LOOP_TIME = 0.02;
        public static final int ODOMETRY_UPDATE_RATE_HZ_INTEGER = 200;
        public static final boolean MODULE_IS_INVERTED = CHOSEN_CONSTANTS.driveMotorInverted;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQ = CHOSEN_CONSTANTS.maxAccelMetersPerSec;
        public static final double SUPPLY_CURRENT_LIMIT = CHOSEN_CONSTANTS.supplyCurrentLimit;
        public static final double STATOR_CURRENT_LIMIT = CHOSEN_CONSTANTS.statorCurrentLimit;

        // front right wheel
        public static final int FRONT_RIGHT_DRIVE_ID = 13;
        public static final int FRONT_RIGHT_TURN_ID = 12;
        public static final int FRONT_RIGHT_CANCODER_ID = 11;
        public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS = CHOSEN_CONSTANTS.frontRightOffset;

        // front left wheel
        public static final int FRONT_LEFT_DRIVE_ID = 23;
        public static final int FRONT_LEFT_TURN_ID = 22;
        public static final int FRONT_LEFT_CANCODER_ID = 21;
        public static final double FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS = CHOSEN_CONSTANTS.frontLeftOffset;

        // back left wheel
        public static final int BACK_LEFT_DRIVE_ID = 33;
        public static final int BACK_LEFT_TURN_ID = 32;
        public static final int BACK_LEFT_CANCODER_ID = 31;
        public static final double BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS = CHOSEN_CONSTANTS.backLeftOffset;

        // back right wheel
        public static final int BACK_RIGHT_DRIVE_ID = 43;
        public static final int BACK_RIGHT_TURN_ID = 42;
        public static final int BACK_RIGHT_CANCODER_ID = 41;
        public static final double BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS = CHOSEN_CONSTANTS.backRightOffset;

        public static final TrapezoidProfile.Constraints TRAPEZOID_THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RAD_PER_SECOND, MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final double TRACK_WIDTH_METERS = CHOSEN_CONSTANTS.trackWidthMeters;
        public static final double WHEEL_BASE_METERS = CHOSEN_CONSTANTS.wheelBaseMeters;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                CHOSEN_CONSTANTS.frontRightTranslation, // Front right wheel
                CHOSEN_CONSTANTS.frontLeftTranslation, // Front left wheel
                CHOSEN_CONSTANTS.backLeftTranslation, // Back left wheel
                CHOSEN_CONSTANTS.backRightTranslation); // Back right wheel
    }

    public static final class Arm{
        public static final double INTAKE_POSITION_RADIANS = Math.toRadians(52.25);//0.99;
        public static final double SHOOT_POSITION_RADIANS = 1.05;




        
        //113.2 degrees (blue end amp pos)
        public static final double AMP_POSITION_RADIANS = Math.toRadians(115.5);// 113.5 //Math.toRadians(117.5);
        
        public static final double STOCKPILE_POSITION_RADIANS = 0.7;
        public static final double READY_POSITION_RADIANS = 0.4;
        public static final double PODIUM_SHOT_RADIANS = Math.toRadians(39);
    }

    public static final class Shooter{
        public static final int STOCKPILE_RPM = 3500;
        public static final double MAX_SHOOTING_DISTANCE = 100;
    }
}
