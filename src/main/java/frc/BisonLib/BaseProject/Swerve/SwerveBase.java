package frc.BisonLib.BaseProject.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.BisonLib.BaseProject.Vision.VisionManagerBase;
import frc.BisonLib.BaseProject.Vision.VisionPosePacket;
import frc.robot.Constants;

import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.studica.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.*;
import com.pathplanner.lib.controllers.*;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveBase extends SubsystemBase {

    protected TalonFXModule[] modules;
    protected final VisionManagerBase visionManager;

    protected final Field2d m_field = new Field2d();
    private final SwerveDrivePoseEstimator odometry;

    // NEVER DIRECTLY CALL ANY GYRO METHODS, ALWAYS USE THE SYNCHRONIZED GYRO LOCK!!
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, Constants.Swerve.ODOMETRY_UPDATE_RATE_HZ_INTEGER);


    // private final BuiltInAccelerometer rioAccelerometer = new BuiltInAccelerometer();
    // private final LinearFilter xAccelFilter = LinearFilter.movingAverage(5);
    // private final LinearFilter yAccelFilter = LinearFilter.movingAverage(5);
    private final PIDController thetaController = new PIDController(Constants.Swerve.ROBOT_ROTATION_KP, 0, 0);
    private final BaseStatusSignal[] allOdomSignals;

    protected double max_accel = 0;
    protected double speed = 0;
    protected boolean rotatedToSetpoint = false;

    // used for wheel characterization
    protected double initialGyroAngle = 0;
    protected double[] initialPositions = new double[4];

    protected double lastTime = Timer.getFPGATimestamp();
    protected double currentTime = 0;
    protected double totalLoopTime = 0;
    protected double inc = 0;

    public final Trigger atRotationSetpoint = new Trigger(()-> robotRotationAtSetpoint());
    public PPHolonomicDriveController pathplannerController =  new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                                                new PIDConstants(6, 0.0, 0.1), // Translation PID constants (JPK was 6,0,0)
                                                                new PIDConstants(5, 0.0, 0.0) // Rotation PID constants (JPK was 2)
                                                                );



    // this is a lock to make sure nobody acesses our pose while odometry is updating it
    private final ReentrantReadWriteLock odometryLock = new ReentrantReadWriteLock();

    // this is a lock to make sure nobody acesses our gyro while odometry is updating it
    private final Object gyroLock = new Object();

    private Pose2d currentRobotPose = new Pose2d();
    private SwerveModulePosition[] currentModulePositions = new SwerveModulePosition[4];
    private SwerveModuleState[] currentModuleStates = new SwerveModuleState[4];

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    /**
     * Does all da constructing
     * 
     * @param cameras An array of cameras used for pose estinmation
     * @param moduleTypes The type of swerve module on the swerve drive
     */
    public SwerveBase(VisionManagerBase visionManager, TalonFXModule[] modules) {
        // 4 modules * 3 signals per module
        allOdomSignals = new BaseStatusSignal[(4 * 3)];
        for(int i = 0; i < modules.length; ++i){
            var signals = modules[i].getOdometrySignals();
            allOdomSignals[i*3 + 0] = signals[0]; // drive position
            allOdomSignals[i*3 + 1] = signals[1]; // drive velocity
            allOdomSignals[i*3 + 2] = signals[2]; // module rotation (cancoder)
        }


        this.visionManager = visionManager;

        // Holds all the modules
        this.modules = modules;

        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
            e.printStackTrace();
            config = new RobotConfig(58.967, 6.883, new ModuleConfig(0.051, 5.3, 1.15, DCMotor.getFalcon500(1), 40, 4), 0.572, 0.572);
        }

        initAutoBuilder(config);


        /*
        * Sets the gyro at the beginning of the match and 
        * also sets each module state to present
        */

        //maybe have this keep trying to reset the gyro if it fails once
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                setGyro(180);
            } catch (Exception e) {
            }
        }).start();


        thetaController.enableContinuousInput(-180, 180);


        odometryLock.writeLock().lock();
        Rotation2d gyroHeading = getGyroHeading();
        currentModulePositions = getModulePositions();
        try{
            odometry = new SwerveDrivePoseEstimator
            (
                Constants.Swerve.kDriveKinematics, 
                gyroHeading,
                currentModulePositions,
                new Pose2d()
            );
        }finally{
            odometryLock.writeLock().unlock();
        }


        setpointGenerator = new SwerveSetpointGenerator(
            config, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
            Units.rotationsToRadians(Constants.Swerve.MAX_WHEEL_ROTATIONAL_SPEED) // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
        );

        
        ChassisSpeeds speeds = new ChassisSpeeds();
        SwerveModuleState[] states = new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
        previousSetpoint = new SwerveSetpoint(speeds, states, DriveFeedforwards.zeros(config.numModules));


        SmartDashboard.putData("field", m_field);
    }


    public void playSong(){
        if(modules[0].getModuleType().equals("TalonFXModule")){
            new Thread(() -> {
                try {
                        Orchestra orchestra = new Orchestra();
                        for(var module : this.modules){
                            orchestra.addInstrument(module.getDriveMotor());
                            orchestra.addInstrument(module.getTurnMotor());
                        }

                        var status  = orchestra.loadMusic("EmpireStrikesBack.chrp");
                        if(status.isOK()){
                            double startTime = Timer.getFPGATimestamp();
                            orchestra.play();
                            while((Timer.getFPGATimestamp() - startTime) < 10){}
                            orchestra.stop();
                        }
                        orchestra.close();
                } catch (Exception e) {
                }
            }).start();
        }
    }


    /*
     * Initializes autobuilder,
     * this is used by pathplanner
     */
    public void initAutoBuilder(RobotConfig config){

        // Configure the AutoBuilder last
        AutoBuilder.configure(
                this::getSavedPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getLatestChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds, false), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                pathplannerController,
                config,
                this::isRedAlliance,
            this // Reference to this subsystem to set requirements
        );
    }


    /*
     * isRedAlliance returns true if we are red alliance and returns false if we are blue alliance
     */
    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                boolean temp = (alliance.get() == DriverStation.Alliance.Red) ? true : false;
                //SmartDasboard.putBoolean("Alliance", temp);
                return temp;
              }
        //SmartDasboard.putBoolean("Alliance", false);
        return false;
    }


    /*
     * Sets all the swerve modules to the states we want them to be in (velocity + angle)
     */
    public void setModules(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS);

        for(var module : modules){
            //SmartDashboard.putString("Swerve/Module State " + module.index, desiredStates[module.index].toString());
            module.setDesiredState(desiredStates[module.index]);
        }
        
    }


    /**
     * setGyro sets the gyro to a given angle,
     * it does this by resetting the gyro to 0and then giving it an offset
     * (negative offset bcs wpilib is ccw+ and navX is cw+)
     * 
     * @param degrees the degree value that the gyro should be set to
     */
    public void setGyro(double degrees){
        synchronized(gyroLock){
            gyro.reset();
            gyro.setAngleAdjustment(-degrees);
        }
    }


    /**
     * checks if the robot is facing the direction of the rotation override
     * 
     * @return if the robot is facing the rotation override angle
     */
    public boolean robotRotationAtSetpoint(){
        return rotatedToSetpoint;
    }


    /**
     * Finds a new angular speed based on rotation override
     * 
     * @param originalSpeeds The original chassis speeds of the robot as inputted by the driver
     * 
     * @return The new rotation component as calculated by the rotation override
     */
    public double getAngularComponentFromRotationOverride(double wantedAngle){
        double currentRotation = getSavedPose().getRotation().getDegrees();
        double pidOutput = thetaController.calculate(currentRotation, wantedAngle);

        //SmartDasboard.putNumber("Swerve/Current Robot Rotation", currentRotation);
        //SmartDasboard.putNumber("Swerve/Setpoint Robot Rotation", wantedAngle);
        //SmartDasboard.putNumber("Swerve/Rotation PID output", pidOutput);

        rotatedToSetpoint = Math.abs(currentRotation - wantedAngle) < 3;
        return MathUtil.clamp(pidOutput, -1, 1) * Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECOND;
    }


    /**
     * This method should not be touched by anything except odometry, 
     * all angles should be pulled from odometry and not directly from the gyro.
     * This is done to ensure that there is no thread contention for this method.
     * 
     * @returns the angle the gyro is facing expressed as a Rotation2d
     */
    private Rotation2d getGyroHeading() {
        synchronized (gyroLock){
            return new Rotation2d(-Math.toRadians(Math.IEEEremainder(gyro.getAngle(), 360)));
        }
    }

    private double getGyroRate() {
        synchronized (gyroLock){
            return gyro.getRate();
        }
    }


    /**
     * calculates the distance from the current robot pose to the supplied translation,
     * can be potentially used for figuring out if the robot is in some specific zone
     * 
     * @param other The translation to find the distance to
     * @return The distance from the robot pose to the other supplied translation
     */
    public double getDistanceToTranslation(Translation2d other){
        return getSavedPose().getTranslation().getDistance(other);
    }


    /*
     * Returns the current chassis speeds of the robot, 
     * used with pathplanner
     */
    public ChassisSpeeds getLatestChassisSpeed(){
        ChassisSpeeds speeds;
        odometryLock.readLock().lock();
        try{
            speeds = Constants.Swerve.kDriveKinematics.toChassisSpeeds(currentModuleStates);
        }finally{
            odometryLock.readLock().unlock();
        }
        return speeds;
    }


    public SwerveModuleState[] getLatestModuleStates(){
        SwerveModuleState[] states;
        odometryLock.readLock().lock();
        try{
            states = currentModuleStates;
        }finally{
            odometryLock.readLock().unlock();
        }
        return states;
    }
    
    

    public double getLatestSpeed(){
        return speed;
    }


    /**
     * @returns an array containing the position of each swerve module (check SwerveModule.java for further details)
     */
    public SwerveModulePosition[] getModulePositions() {

        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        for(var module : modules){
            modulePositions[module.index] = module.getPosition();
        }

        return modulePositions;
    }


    /**
     * @return an array containing the state (velocity + rotation) of each swerve module
     */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];

        for(var module : modules){
            states[module.index] = module.getState();
        }

        return states;
    }


    /**
     * Gets the acceleration of each swerve module, 
     * keep in mind these can be slightly inaccurate because of wheel slippage
     * 
     * @return a double array that contains the acceleration of each swerve module
     */
    public double[] getModuleAccelerations(){
        double[] accelerations = new double[4];

        for(var module : modules){
            accelerations[module.index] = module.getDriveAcceleration();
        }

        return accelerations;
    }


    public double[] getRawDrivePositions(){
        double[] positions = new double[4];

        for(var module : modules) {
            positions[module.index] = module.getUncachedDrivePosition();
        }

        return positions;
    }


    /**
     * Manually resets the odometry to a given pose
     * Also resets the gyro
     * Pretty much only used at the start of auton
     * 
     * @param pose The pose to set the robot pose to
     */
    public void resetOdometry(Pose2d pose) {
        setGyro(pose.getRotation().getDegrees());
        Rotation2d gyroHeading = getGyroHeading();
        odometryLock.writeLock().lock();
        try{
            odometry.resetPosition(
                gyroHeading,
                currentModulePositions,
                pose);
        }finally{
            odometryLock.writeLock().unlock();
        }
    }


    /**
     * resetGyro sets the gyro to "facing away from the driver station"
     * 
     * @return A command that "zeroes" our gyro and syncs our swerve modules
     */
    public Command resetGyro() {
        return runOnce(
            ()-> 
                {
                    if(isRedAlliance()){
                        odometryLock.writeLock().lock();
                        try{
                            resetOdometry(new Pose2d(currentRobotPose.getX(), currentRobotPose.getY(), new Rotation2d(Math.PI)));
                        }finally{
                            odometryLock.writeLock().unlock();
                        }
                    }
                    else {
                        odometryLock.writeLock().lock();
                        try{
                            resetOdometry(new Pose2d(currentRobotPose.getX(), currentRobotPose.getY(), new Rotation2d(0)));
                        }finally{
                            odometryLock.writeLock().unlock();
                        }
                    }
                }
        );
    }

    /*
     * Calculates the circumference of the wheel by turning in place slowly
     * 
     * COMMENT OUT DRIVEBASE AND ODOMETRY CODE BEFORE RUNNING THIS
     */
    public Command runWheelCharacterization(){

        // total distance each module should travel for one rotation
        // circumferenc = pi * d
        double one_rotation_distance = Constants.Swerve.WHEEL_BASE_METERS * Math.PI * Math.sqrt(2);

        return runOnce(()-> {

            //get each module in positions
            for(var mod : modules){
                    mod.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45 + mod.index * 90)));
            }

        })
        .andThen(
         waitSeconds(0.5)
        )
        .andThen(runOnce(
          ()-> {
            initialPositions = getRawDrivePositions();
            initialGyroAngle = gyro.getAngle() + 180;
          }
        ))
        .andThen(
            deadline(
                new WaitCommand(6)
                ,
                run(()-> {
                    // closed loop control to turn in place one rotation
                    for(var mod : modules){
                        mod.setDesiredState(new SwerveModuleState(0.3, Rotation2d.fromDegrees(45 + mod.index * 90)));
                    }
                })
                )
        )
        .andThen(
            waitSeconds(1)
        )
        .andThen(
            runOnce(()-> {
                double[] currentPositions = getRawDrivePositions();
                double avg_calculated_wheel_circumference = 0;
                double actual_distance_traveled = one_rotation_distance * Math.abs(gyro.getAngle()-initialGyroAngle)/360.;
                for(var mod : modules){
                    //original_circumference/new_circumference = calculated_distance/actual_distance
                    // actual_distance * original_circumference = new_circumference * calculated_distance
                    // new_circumference = actual_distance * original_circumference / (calculated_distance)
                    double new_circumference = actual_distance_traveled * Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS / (initialPositions[mod.index] - currentPositions[mod.index]);
                    avg_calculated_wheel_circumference += new_circumference;
                    SmartDashboard.putNumber("initial distance " + mod.index, initialPositions[mod.index]);
                    SmartDashboard.putNumber("current distance " + mod.index, currentPositions[mod.index]);
                    SmartDashboard.putNumber("Swerve/Module " + mod.index + "/calculated wheel circumference", new_circumference);
                }
                avg_calculated_wheel_circumference /= 4;
                SmartDashboard.putNumber("Swerve/Average Calculated Wheel Circumference", avg_calculated_wheel_circumference);
            })
        )
        ;
    }
    

    /*
     * Stops all of the swerve modules
     */
    public void stopModules() {
        for(var module : modules){
            module.stop();
        }
    }


    /**
     * Continuously rotates robot to the specified angle while maintaining normal driver control of the robot
     * 
     * @param angleDegrees The angle in degrees that the robot should turn to, 
     *                     this is a double supplier so you can continously pass different values
     * 
     * @return Returns a functional command that will rotate the robot to a specified angle, 
     *         when interrupted, will return driver control to robot rotation
     */
    public Command rotateToAngle(DoubleSupplier angleDegrees, Supplier<ChassisSpeeds> speedSupplier){
        return run
        (
            /* EXCECUTE */
            ()-> {
                    ChassisSpeeds speeds = speedSupplier.get();
                    speeds.omegaRadiansPerSecond = getAngularComponentFromRotationOverride(angleDegrees.getAsDouble());
                    drive(speeds, true);
                 }

            // /* INTERRUPTED */
            // interrupted-> {
            //                 setStateToDriverControl();
            //                 rotatedToSetpoint = false;
            //               },
        );
    }


    public Command driveToPose(Pose2d targetPose, double endVelocity){
        PathConstraints constraints = new PathConstraints(
            Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS, Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ,
            Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECOND, Constants.Swerve.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED
        );

        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            endVelocity // Goal end velocity in meters/sec
        );

        return pathfindingCommand;
    }

    /**
     * Drives swerve given chassis speeds robot relative
     * 
     * @param chassisSpeeds The chassis speeds the robot should travel at
     */
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds, boolean useSetpointGenerator) {
        if(!useSetpointGenerator){
            // discretizes the chassis speeds (acccounts for robot skew)
            chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, Constants.Swerve.ROBOT_LOOP_TIME);

            //SmartDashboard.putString("Swerve/Commanded Chassis Speeds", chassisSpeeds.toString());
            // convert chassis speeds to module states
            SwerveModuleState[] moduleStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            // set the modules to their desired speeds
            setModules(moduleStates);
        }
        else{
            previousSetpoint = setpointGenerator.generateSetpoint(
                previousSetpoint, // The previous setpoint
                chassisSpeeds, // The desired target speeds
                Constants.Swerve.ROBOT_LOOP_TIME // The loop time of the robot code, in seconds
            );

            //set the modules to their desired speeds
            setModules(previousSetpoint.moduleStates());
        }
    }


    /**
     * THIS DOESN'T WORK!!!!!!
     * but it would be really cool if it did, maybe if someone has time then they fix it!?
     * 
     * @param chassisSpeeds The chassis speeds the robot should drive with
     */
    public void driveSwerveFromChassisSpeedsCustomCenterOfRotation(ChassisSpeeds chassisSpeeds){

        // discretizes the chassis speeds (acccounts for robot skew)
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, Constants.Swerve.ROBOT_LOOP_TIME);
        //SmartDasboard.putNumber("Swerve/chassis x", chassisSpeeds.vxMetersPerSecond);
        //SmartDasboard.putNumber("Swerve/chassis y", chassisSpeeds.vyMetersPerSecond);
        //SmartDasboard.putNumber("Swerve/chassis omega", chassisSpeeds.omegaRadiansPerSecond);


        // I derivated whole thing using polar coordinates but the Translation2d turns it back into standard x, y coordinates
        // Here is my work, I didn't write this in a way that I mean't to be easy to be understood by others but make of it what you will
        // https://i.imgur.com/jL4c0yS.jpeg
        double radius = Math.sqrt(2) * Constants.Swerve.WHEEL_BASE_METERS/2.0;
        double offset;

        // Depending on which direction we want to rotate we choose a different center of rotation
        if(chassisSpeeds.omegaRadiansPerSecond > 0){
            offset = Math.PI/4;
        }
        else{
            offset = -Math.PI/4;
        }
        SwerveModuleState[] moduleStates = 
                Constants.Swerve.kDriveKinematics.toSwerveModuleStates(
                    chassisSpeeds,
                    new Translation2d(
                        radius,
                        new Rotation2d(Math.atan(chassisSpeeds.vxMetersPerSecond / chassisSpeeds.vyMetersPerSecond) + offset)
                    )
                );

        // set the modules to their desired speeds
        setModules(moduleStates);
    }

    /*
     * Drives the robot in teleop, we don't want it fighting the auton swerve commands
     */
    public void teleopDefaultCommand(Supplier<ChassisSpeeds> speedsSupplier, boolean fieldOriented){
        // We don't want to run the TELEOP default command in auton
        if(DriverStation.isAutonomous()){
            return;
        }

        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speedsSupplier.get(), getSavedPose().getRotation()), false);
    }
    
    /**
     * Drives swerve given chassis speeds
     * Should be called every loop
     * 
     * @param speeds the commanded chassis speeds from the joysticks
     * @param fieldOriented A boolean that specifies if the robot should be driven in fieldOriented mode or not
     */
    public void drive(ChassisSpeeds speeds, boolean fieldOriented){

        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getSavedPose().getRotation());
        }

        this.driveRobotRelative(speeds, false);
    }


    /*
     * updateOdometryWithVision uses vision to add measurements to the odometry
     */
    public void updateOdometryWithVision(){
        VisionPosePacket[] poses = visionManager.getPoses();

        for(var pose : poses){

            // Only update pose if it is valid and if we arent spinning too fast
            if(pose.isValidPose && getGyroRate() < 720){

                // Finally, we actually add the measurement to our odometry
                odometryLock.writeLock().lock();
                try{
                    odometry.addVisionMeasurement
                    (
                        pose.pose, 
                        pose.timestamp,
                        
                        // This way it doesn't trust the rotation reading from the vision
                        VecBuilder.fill(pose.stdDev, pose.stdDev, 999999)
                    );
                }finally{
                    odometryLock.writeLock().unlock();
                }
                
                // This puts the pose reading from each camera onto the Field2d Widget,
                // Docs - https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html
                m_field.getObject(pose.name).setPose(pose.pose);
            }
        }  
    }


    public void updateOdometryWithKinematics(){
        // lastTime = currentTime;
        // currentTime = Timer.getFPGATimestamp();
        // inc++;
        // totalLoopTime += (currentTime-lastTime);
        //SmartDashboard.putNumber("avg loop time", totalLoopTime/inc);

        StatusCode signal = BaseStatusSignal.waitForAll(2 / Constants.Swerve.ODOMETRY_UPDATE_RATE_HZ_INTEGER, allOdomSignals);
        if(signal.isError()) System.out.println(signal);

        SwerveModulePosition[] positions = getModulePositions();
        SwerveModuleState[] states = getModuleStates();
        odometryLock.writeLock().lock();
        try{
            currentModulePositions = positions;
            currentRobotPose = odometry.updateWithTime(
                Timer.getFPGATimestamp(),
                getGyroHeading(),
                currentModulePositions);
            currentModuleStates = states;
        }finally{
            odometryLock.writeLock().unlock();
        }
    }


    /**
     * Used to find the max stator current to prevent wheel slip
     * https://pro.docs.ctr-electronics.com/en/latest/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
     * 
     * 
     * @param voltageSupplier
     */
    public void testModules(DoubleSupplier voltageSupplier){
        //SmartDasboard.putNumber("Swerve/Module 1/Module 1 Current", modules[0].getDriveStatorCurrent());
        //SmartDashboard.putNumber("Swerve/Module 2/Module 2 Current", modules[1].getDriveStatorCurrent());
        //SmartDashboard.putNumber("Swerve/Module 3/Module 3 Current", modules[2].getDriveStatorCurrent());
        //SmartDashboard.putNumber("Swerve/Module 4/Module 4 Current", modules[3].getDriveStatorCurrent());

        for(var mod : modules){
            mod.driveWithVoltage(voltageSupplier.getAsDouble());
        }
    }

    public Pose2d getSavedPose(){
        Pose2d pose;
        odometryLock.readLock().lock();
        try{
            pose = currentRobotPose;
        }finally{
            odometryLock.readLock().unlock();
        }
        return pose;
    }


    @Override
    public void periodic() {
        //modules[0].setTurnMotor(1);
        //SmartDashboard.putNumber("turn motor speed", modules[0].getTurnVelocity());

        // ChassisSpeeds currentSpeeds = getLatestChassisSpeed();
        // speed = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        //double navXAccel = Math.hypot(gyro.getWorldLinearAccelX(), gyro.getWorldLinearAccelY());
        //double rioAccel = Math.hypot(xAccelFilter.calculate(rioAccelerometer.getX()), yAccelFilter.calculate(rioAccelerometer.getY()));
        // if(rioAccel > max_accel){
        //     max_accel = rioAccel;
        // }

        //updateOdometryWithKinematics();
        //updateOdometryWithVision();


       //SmartDashboard.putNumber("Actual Gyro Update Rate", gyro.getActualUpdateRate());
        m_field.setRobotPose(getSavedPose());
         

        // SwerveModuleState[] modStates = getModuleStates();
        // double[] modAccelerations = getModuleAccelerations();

        // //SmartDashboard.putNumber("Swerve/Module 1/Module 1 Angle rad", modStates[0].angle.getRadians());
        // //SmartDashboard.putNumber("Swerve/Module 2/Module 2 Angle rad", modStates[1].angle.getRadians());
        // //SmartDashboard.putNumber("Swerve/Module 3/Module 3 Angle rad", modStates[2].angle.getRadians());
        // //SmartDashboard.putNumber("Swerve/Module 4/Module 4 Angle rad", modStates[3].angle.getRadians());

        // SmartDashboard.putNumber("Swerve/Module 1/Module 1 Velocity", modStates[0].speedMetersPerSecond);
        // SmartDashboard.putNumber("Swerve/Module 2/Module 2 Velocity", modStates[1].speedMetersPerSecond);
        // SmartDashboard.putNumber("Swerve/Module 3/Module 3 Velocity", modStates[2].speedMetersPerSecond);
        // SmartDashboard.putNumber("Swerve/Module 4/Module 4 Velocity", modStates[3].speedMetersPerSecond);

        // SmartDashboard.putNumber("Swerve/Module 1/Module 1 Accel", modAccelerations[0]);
        // SmartDashboard.putNumber("Swerve/Module 2/Module 2 Accel", modAccelerations[1]);
        // SmartDashboard.putNumber("Swerve/Module 3/Module 3 Accel", modAccelerations[2]);
        // SmartDashboard.putNumber("Swerve/Module 4/Module 4 Accel", modAccelerations[3]);

        // //SmartDashboard.putNumber("Swerve/current robot velocity", speed);
        // // //SmartDashboard.putNumber("Swerve/max rio measured acceleration", max_accel);
        // // //SmartDashboard.putNumber("Swerve/Current NavX measured acceleration", navXAccel);
        // // //SmartDashboard.putNumber("Swerve/Current Rio acceleration", rioAccel);
        
        SmartDashboard.putBoolean("Robot Rotation at Setpoint", atRotationSetpoint.getAsBoolean());
    }
}

