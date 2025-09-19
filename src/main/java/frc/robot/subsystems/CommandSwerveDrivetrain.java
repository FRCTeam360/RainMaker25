package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.OldCompBot;
import frc.robot.generated.OldCompBot.TunerSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionMeasurement;
import frc.robot.utils.CommandLogger;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private final String CMD_NAME = "Swerve: ";
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public PhoenixPIDController headingController;
    public PhoenixPIDController strafeController;
    public PhoenixPIDController forwardController;
    public PhoenixPIDController poseXController;
    public PhoenixPIDController poseYController;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public final Command fieldOrientedDrive(
            CommandXboxController driveCont) { // field oriented drive command!
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() // creates a fieldcentric drive
                .withDeadband(maxSpeed * 0.01)
                .withRotationalDeadband(maxAngularRate * 0.01);
                // .withDriveRequestType(DriveRequestType.Velocity); // Use closed-loop control for drive motors

        return CommandLogger.logCommand(this.applyRequest(
                () -> drive
                        .withVelocityX(
                                Math.pow(driveCont.getLeftY(), 3) *
                                        maxSpeed *
                                        -1.0) // Drive forward with negative Y (forward)
                        .withVelocityY(
                                Math.pow(driveCont.getLeftX(), 3) *
                                        maxSpeed *
                                        -1.0) // Drive left with negative X (left)
                        .withRotationalRate(
                                Math.pow(driveCont.getRightX(), 2) *
                                        (maxAngularRate / 2.0) * -Math.signum(driveCont.getRightX())) // Drive
                                                                                                      // counterclockwise
                                                                                                      // with negative X
                                                                                                      // (left)
        ), "DrivetrainFieldOriented");
    }

    public final Command rotateDrivetrain() { // field oriented drive command!
    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric(); // creates a fieldcentric drive
            // .withDriveRequestType(DriveRequestType.Velocity); // Use closed-loop control for drive motors

    return CommandLogger.logCommand(this.applyRequest(
            () -> drive
                    .withVelocityX(0.0) // Drive forward with negative Y (forward)
                    .withVelocityY(0.0) // Drive left with negative X (left)
                    .withRotationalRate(0.5 * (maxAngularRate / 2.0)) // Drive                                    // (left)
    ), "rotateDrivetrain");
}


    public void xOut() {
        SwerveRequest xOutReq = new SwerveRequest.SwerveDriveBrake();
        this.setControl(xOutReq);
    }

    public Command xOutCmd() {
        SwerveRequest xOutReq = new SwerveRequest.SwerveDriveBrake();
        return CommandLogger.logCommand(this.applyRequest(() -> xOutReq), "DrivetrainXOut");
    }

    public final void zero() {
        this.tareEverything();
    }

    public void addHeadingController(double kP, double kI, double kD, double kIZone) {
        headingController = new PhoenixPIDController(kP, kI, kD);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(1.0));
    }

    public void addStrafeController(double kP, double kI, double kD, double irMax, double irMin) {
        strafeController = new PhoenixPIDController(kP, kI, kD);
        strafeController.setTolerance(0.5);
    }

    public void addForwardContrller(double kP, double kI, double kD, double irMax, double irMin) {
        forwardController = new PhoenixPIDController(kP, kI, kD);
        forwardController.setTolerance(0.75);
    }

    public void driveFieldCentricFacingAngle(double x, double y, double desiredAngle) {
        FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(x * maxSpeed)
                .withVelocityY(y * maxSpeed)
                .withTargetDirection(Rotation2d.fromDegrees(desiredAngle));
        request.HeadingController = headingController;
        request.withDeadband(0.1);
        request.withRotationalDeadband(0.0001);
        this.setControl(request);
        request.withDriveRequestType(DriveRequestType.Velocity);
    }

    public void bargeFieldCentricFacingAngle(double x, double y, double desiredAngle) {
        FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(x * maxSpeed)
                .withVelocityY(y * maxSpeed)
                .withTargetDirection(Rotation2d.fromDegrees(desiredAngle));
        request.HeadingController = new PhoenixPIDController(1.7, 0, 0);
        request.withDeadband(0.1);
        request.withRotationalDeadband(0.0001);
        this.setControl(request);
        request.withDriveRequestType(DriveRequestType.Velocity);
    }

    public void driveFieldCentricFacingAngleBluePerspective(double x, double y, double desiredAngle) {
        FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(x * maxSpeed)
                .withVelocityY(y * maxSpeed)
                .withTargetDirection(Rotation2d.fromDegrees(desiredAngle));
        request.HeadingController = headingController;
        request.withDeadband(0.1);
        request.withRotationalDeadband(0.021); //used to be 0.04
        request.ForwardPerspective = ForwardPerspectiveValue.BlueAlliance;
        this.setControl(request);
        request.withDriveRequestType(DriveRequestType.Velocity);
    }



    // public Command backUpBot(double meters) {
    // return Commands.runOnce(() -> this.start)
    // }

    public void robotCentricDrive(double x, double y, double rotation) {
        this.setControl(
                new SwerveRequest.RobotCentric()
                        .withVelocityX(x * maxSpeed)
                        .withVelocityY(y * maxSpeed)
                        .withRotationalRate(-rotation * maxAngularRate));
    }

    public final Command robotCentricDrive(
            CommandXboxController driveCont) { // field oriented drive command!
        SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric() // creates a fieldcentric drive
                .withDeadband(maxSpeed * 0.1)
                .withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        return this.applyRequest(
                () -> drive
                        .withVelocityX(
                                Math.pow(driveCont.getLeftY(), 3) *
                                        maxSpeed *
                                        -Math.signum(driveCont.getLeftY())) // Drive forward with negative Y (forward)
                        .withVelocityY(
                                Math.pow(driveCont.getLeftX(), 3) *
                                        maxSpeed *
                                        -Math.signum(driveCont.getLeftX())) // Drive left with negative X (left)
                        .withRotationalRate(
                                Math.pow(driveCont.getRightX(), 3) *
                                        maxAngularRate *
                                        Math.signum(driveCont.getRightX())) // Drive counterclockwise with negative X
                                                                            // (left)
        );
    }

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

    private double maxSpeed;
    private double maxAngularRate;

    public double getMaxSpeed(){
        return maxSpeed;
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            double headingKP,
            double headingKI,
            double headingKD,
            double headingKIZone,
            double stafeKP,
            double stafeKI,
            double stafeKD,
            double strafeIRMax,
            double strafeIRMin,
            double forwardKP,
            double forwardKI,
            double forwardKD,
            double forwardIRMax,
            double forwardIRMin,
            PhoenixPIDController poseXController,
            PhoenixPIDController poseYController,
            double maxSpeed,
            double maxAngularRate,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        System.out.println("Utils.isSimulation() " + Utils.isSimulation());
        if (Utils.isSimulation()) {
            startSimThread();
        }
        addHeadingController(headingKP, headingKI, headingKD, headingKIZone);
        addStrafeController(stafeKP, stafeKI, stafeKD, strafeIRMax, strafeIRMin);
        addForwardContrller(forwardKP, forwardKI, forwardKD, forwardIRMax, forwardIRMin);
        this.poseXController = poseXController;
        this.poseYController = poseYController;

        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;

        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                odometryStandardDeviation,
                visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        long executeStartTime = HALUtil.getFPGATime();
        long executeLoopTime = HALUtil.getFPGATime() - executeStartTime;
        Logger.recordOutput(CMD_NAME + "execute loop time", (executeLoopTime / 1000.0));

        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public Pose2d getPose() {
        return this.getStateCopy().Pose;
    }

    public Rotation2d getRotation2d() {
        return this.getPose().getRotation();
    }

    public double getAngle() {
        return this.getRotation2d().getDegrees();
    }

    /**
     * Checks the heading controller for the drivetrain's rotation
     * 
     * @return true if the drivetrain has reached its rotation setpoint within
     *         tolerance
     */
    public boolean isAtRotationSetpoint() {
        return headingController.atSetpoint();
    }

    public void setSteerCoast(boolean isCoast) {
        if (isCoast) {
            for (int i = 0; i < 4; i++) {
                this.getModule(i).getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
            }
        } else {
            for (int i = 0; i < 4; i++) {
                this.getModule(i).getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
            }
        }
    }

    /**
     * A command that waits until the drivebase is facing the setpoint angle
     * 
     * @return A wait until command that ends when the drivetrain is facing its
     *         rotational setpoint
     */
    public Command waitUntilDrivetrainAtHeadingSetpoint() {
        return Commands.waitUntil(() -> isAtRotationSetpoint());
    }

    
    public double getAngularRate() {
        return Math.toDegrees(this.getStateCopy().Speeds.omegaRadiansPerSecond);
    }

    public double getXRate() {
        return this.getStateCopy().Speeds.vxMetersPerSecond;
    }

    public double getYRate() {
        return this.getStateCopy().Speeds.vyMetersPerSecond;
    }

    public void addVisionMeasurements(List<VisionMeasurement> measurements) {
        for (VisionMeasurement measurement : measurements) {
            this.addVisionMeasurement(
                    measurement.estimatedPose(),
                    Utils.fpgaToCurrentTime(measurement.timestamp()),
                    measurement.standardDeviation());
        }
    }

    @Override
    public void periodic() {
        long periodicStartTime = HALUtil.getFPGATime();
        
        Logger.recordOutput(CMD_NAME+ " Current Pose", this.getPose());
        // Logger.recordOutput("Swerve: Rotation", this.getRotation2d());
        // Logger.recordOutput("Swerve: Angle", this.getAngle());
        // Logger.recordOutput("swerve: pithc", this.isFlat());
        // Logger.recordOutput("Rotation2d", this.getPigeon2().getRotation2d());
        Logger.recordOutput(
                CMD_NAME+ "Heading Controller: Setpoint",
                headingController.getSetpoint());
        Logger.recordOutput(
                CMD_NAME+ "Heading Controller: Error",
                headingController.getPositionError());
        Logger.recordOutput(
                CMD_NAME+ "Heading Controller: AtSetpoint",
                headingController.atSetpoint());
        Logger.recordOutput(
                CMD_NAME+ "Heading Controller: PositionTolerance",
                headingController.getPositionTolerance());
        // Logger.recordOutput("Swerve: CurrentState", this.getStateCopy().ModuleStates);
        // Logger.recordOutput("Swerve: TargetState", this.getStateCopy().ModuleTargets);

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation
                    .getAlliance()
                    .ifPresent(
                            allianceColor -> {
                                setOperatorPerspectiveForward(
                                        allianceColor == Alliance.Red
                                                ? kRedAlliancePerspectiveRotation
                                                : kBlueAlliancePerspectiveRotation);
                                m_hasAppliedOperatorPerspective = true;
                            });
        }

        // if (DriverStation.isDisabled() && DriverStation.isAutonomous()) {
        //     if(!rotationalResetTimer.isRunning()) {

        //         rotationalResetTimer.restart();
        //     } else if (rotationalResetTimer.hasElapsed(5.0)) {

        //         this.initializeRotationForAlliance();
        //         rotationalResetTimer.restart();
        //     } 
        // } else if(rotationalResetTimer.isRunning()) {

        //     rotationalResetTimer.stop();
        //     rotationalResetTimer.reset();
        // }

        long periodicLoopTime = HALUtil.getFPGATime() - periodicStartTime;
        Logger.recordOutput( CMD_NAME+ " periodic loop time", (periodicLoopTime / 1000.0));
    }
    private Timer rotationalResetTimer = new Timer();


    // public void initializeRotationForAlliance() {
    //     Pose2d currentPose = this.getPose();
    //     if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
    //         currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), Rotation2d.kZero);
    //     } else if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
    //         currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), Rotation2d.k180deg);
    //     }
    //     this.resetPose(currentPose);
    // }
        
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(
                () -> {
                    final double currentTime = Utils.getCurrentTimeSeconds();
                    double deltaTime = currentTime - m_lastSimTime;
                    m_lastSimTime = currentTime;

                    /* use the measured time delta, get battery voltage from WPILib */
                    updateSimState(deltaTime, RobotController.getBatteryVoltage());
                });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private void configureAutoBuilder() {
        try {
            // Load the RobotConfig from the GUI settings. You should probably
            // store this in your Constants file
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    () -> getStateCopy().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getStateCopy().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> this.setControl(this.driveRobotRelativeRequest(speeds, feedforwards)),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(11, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(9, 0, 0)),
                    config,
                    // For our team, the path does not need to be flipped for Red vs Blue.
                    // The reasoning for this is that the fields are not constructed the same for
                    // each event, each side is a bit different.
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if(alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Blue;
                        }
                        return false;
                    },
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    private SwerveRequest driveRobotRelativeRequest(
            ChassisSpeeds speeds,
            DriveFeedforwards feedforwards) {
        return new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(speeds)
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                .withDriveRequestType(DriveRequestType.Velocity);
    }

    public void initializeRotationForAlliance(){
        Pose2d currentPose = this.getPose();
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), Rotation2d.kZero);
        } else if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), Rotation2d.k180deg);
        }
        this.resetPose(currentPose);
    }

    public double getHeadingControllerSetpoint(){
        return headingController.getSetpoint();
    }

    public double getHeadingControllerPositionError(){
        return headingController.getPositionError();
    }

    public double getHeadingControllerVelocityError(){
        return headingController.getVelocityError();
    }

    public double getPoseXSetpoint(){
        return poseXController.getSetpoint();
    }

    public boolean isAtPoseXSetpoint(){
        return poseXController.atSetpoint();
    }

    public double getPoseXControllerPositionError(){
        return poseXController.getPositionError();
    }

    public double getPoseXControllerVelocityError(){
        return poseXController.getVelocityError();
    }

    public double getPoseYSetpoint(){
        return poseYController.getSetpoint();
    }

    public boolean isAtPoseYSetpoint(){
        return poseYController.atSetpoint();
    }

    public double getPoseYControllerPositionError(){
        return poseYController.getPositionError();
    }

    public double getPoseYControllerVelocityError(){
        return poseYController.getVelocityError();
    }

    /**
     * Field centric facing angle command without flipping based on operator perspective
     *
     * @param setpointPose the position on the field to move to
     */
    public void driveToPose(Pose2d setpointPose) {
        Pose2d currentPose = getPose();
        double timestamp = getStateCopy().Timestamp;
        double x = poseXController.calculate(currentPose.getX(), setpointPose.getX(), timestamp);
        double y = poseYController.calculate(currentPose.getY(), setpointPose.getY(), timestamp);

        FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(x * maxSpeed)
                .withVelocityY(y * maxSpeed)
                .withTargetDirection(setpointPose.getRotation());
        request.HeadingController = headingController;
        request.withDeadband(0.025);
        request.withRotationalDeadband(0.021);
        request.ForwardPerspective = ForwardPerspectiveValue.BlueAlliance;
        request.withDriveRequestType(DriveRequestType.Velocity);
        this.setControl(request);
    }

}
