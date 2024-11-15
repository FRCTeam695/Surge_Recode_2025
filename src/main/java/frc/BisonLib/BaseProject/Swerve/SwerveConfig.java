package frc.BisonLib.BaseProject.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConfig {

        /*
         * Find these by aligning modules VERY accurately
         */
        public final double frontRightOffset;
        public final double frontLeftOffset;
        public final double backLeftOffset;
        public final double backRightOffset;

        public final double drivingGearRatio;
        public final double turningGearRatio;
        public final double wheelKP;
        public final double wheelKS;
        public final double wheelKD;
        public final double pathplannerOmegaKP;
        public final double pathplannerTranslationKP;
        public final double rotationOverrideKP;
        public final double maxSpeedMetersPerSec;
        public final double maxAccelMetersPerSec;
        
        public final double maxAngularSpeedRadPerSec;
        public final double maxAngularAccelerationRadPerSecondSquared;
        public final double wheelCircumferenceMeters;
        public final double wheelBaseMeters;
        public final double trackWidthMeters;
        public final double statorCurrentLimit;
        public final double supplyCurrentLimit;

        public final Translation2d frontLeftTranslation;
        public final Translation2d frontRightTranslation;
        public final Translation2d backRightTranslation;
        public final Translation2d backLeftTranslation;

        public final boolean driveMotorInverted;


        /**
         * @param frontRightOffset The offset for the front right absolute encoder
         * @param frontLeftOffset The offset for the front left absolute encoder
         * @param backLeftOffset The offset for the back left absolute encoder
         * @param backRightOffset The offset for the back right absolute encoder
         * @param drivingGearRatio The driving gear ratio
         * @param maxSpeedFeetPerSec The maximum speed of the robot in feet per second
         * @param wheelCircumferenceInches The wheel circumference in inches
         * @param wheelKP The KP value for turning the swerve wheel
         * @param profiledKPvalPathplanner The KP value for pathplanner when rotating the robot
         * @param wheelbaseInches The wheel base of the robot in inches
         * @param trackwidthInches The track width of the robot in inches
         * @param maxAngularAccelerationRadPerSecondSquared The maximum angular acceleration in rad/sec^2
         * @param turningGearRatio The gear ratio for turning the robot
         * @param driveMotorInverted If the drive motor is inverted or not
         * @param rotationToAngleKPval The KP value for rotating the robot to an angle (rotation override)
         * @param statorCurrentLimit The stator current limit for the drive motors
         * @param supplyCurrentLimit The supply current limit for the drive motors
         */
        public SwerveConfig(double frontRightOffset, double frontLeftOffset, double backLeftOffset, double backRightOffset, 
                            double drivingGearRatio, double maxSpeedFeetPerSec, double wheelCircumferenceInches, double wheelKP,
                            double profiledKPvalPathplanner, double wheelbaseInches, double trackwidthInches, double maxAngularAccelerationRadPerSecondSquared,
                            double turningGearRatio, boolean driveMotorInverted, double rotationToAngleKPval, double pathplannerTranslationKP,
                            double maxAccelFeetPerSec, double statorCurrentLimit, double supplyCurrentLimit, double wheelKD, double wheelKS){
            
            this.frontRightOffset = frontRightOffset;
            this.frontLeftOffset = frontLeftOffset;
            this.backLeftOffset = backLeftOffset;
            this.backRightOffset = backRightOffset;

            this.wheelBaseMeters = Units.inchesToMeters(wheelbaseInches);
            this.trackWidthMeters = Units.inchesToMeters(trackwidthInches);

            this.frontRightTranslation = new Translation2d(wheelBaseMeters / 2, -trackWidthMeters / 2);
            this.frontLeftTranslation = new Translation2d(wheelBaseMeters / 2, trackWidthMeters / 2);
            this.backRightTranslation = new Translation2d(-wheelBaseMeters / 2, -trackWidthMeters / 2);
            this.backLeftTranslation = new Translation2d(-wheelBaseMeters / 2, trackWidthMeters / 2);

            this.turningGearRatio = turningGearRatio;
            this.drivingGearRatio = drivingGearRatio;
            this.wheelCircumferenceMeters = Units.inchesToMeters(wheelCircumferenceInches);
            this.maxSpeedMetersPerSec = Units.feetToMeters(maxSpeedFeetPerSec);
            this.maxAccelMetersPerSec = Units.feetToMeters(maxAccelFeetPerSec);

            // omega = v/r 
            // note: this doesn't work if the chassis isn't square
            this.maxAngularSpeedRadPerSec = this.maxSpeedMetersPerSec / this.frontRightTranslation.getNorm();

            this.maxAngularAccelerationRadPerSecondSquared = this.maxAccelMetersPerSec / this.frontRightTranslation.getNorm();
            //this.maxAngularAccelerationRadPerSecondSquared = maxAngularAccelerationRadPerSecondSquared;

            this.wheelKP = wheelKP;
            this.wheelKD = wheelKD;
            this.wheelKS = wheelKS;
            this.pathplannerOmegaKP = profiledKPvalPathplanner;
            this.rotationOverrideKP = rotationToAngleKPval;
            this.pathplannerTranslationKP = pathplannerTranslationKP;
            this.driveMotorInverted = driveMotorInverted;
            this.statorCurrentLimit = statorCurrentLimit;
            this.supplyCurrentLimit = supplyCurrentLimit;
        }
}
