package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
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
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;
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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.OldCompBot;
import frc.robot.generated.OldCompBot.TunerSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionMeasurement;
import frc.robot.utils.LimelightHelpers;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public PhoenixPIDController headingController;
    public PhoenixPIDController strafeController;
    public PhoenixPIDController forwardController;

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
                .withDeadband(maxSpeed * 0.1)
                .withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

        return this.applyRequest(
                () -> drive
                        .withVelocityX(
                                Math.pow(driveCont.getLeftY(), 2) *
                                        maxSpeed *
                                        -Math.signum(driveCont.getLeftY())) // Drive forward with negative Y (forward)
                        .withVelocityY(
                                Math.pow(driveCont.getLeftX(), 2) *
                                        maxSpeed *
                                        -Math.signum(driveCont.getLeftX())) // Drive left with negative X (left)
                        .withRotationalRate(
                                Math.pow(driveCont.getRightX(), 2) *
                                        maxAngularRate *
                                        Math.signum(driveCont.getRightX())) // Drive counterclockwise with negative X
                                                                            // (left)
        );
    }

    public final Command PathOnTheFly(Vision vision, boolean right) {

        final Map<Integer, Double> tagIDToRotation = Map.ofEntries(
                Map.entry(21, 180.0),
                Map.entry(7, 180.0),
                Map.entry(22, 120.0),
                Map.entry(6, 120.0),
                Map.entry(17, 60.0),
                Map.entry(11, 60.0),
                Map.entry(18, 0.0),
                Map.entry(10, 0.0),
                Map.entry(19, -60.0),
                Map.entry(9, -60.0),
                Map.entry(20, -120.0),
                Map.entry(8, -120.0));

        final Map<Integer, List<Waypoint>> LeftTagIDToPosition = Map.ofEntries(

        //ADD SELECT COMMAND AND COLLISION AVOIDENCE STUFFFF
                Map.entry(
                        21,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(6.100, 3.800, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        7,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(14.623, 3.809, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        6,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(13.628, 2.612, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        17,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(3.471, 2.718, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        11,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(11.955, 2.871, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        18,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(2.930, 4.175, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        10,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(11.522, 4.371, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        19,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(3.832, 5.482, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        9,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(12.503, 5.453, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        20,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(5.529, 5.167, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        8,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(14.176, 5.236, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        22,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(5.529, 2.778, Rotation2d.fromDegrees(-33.917)))));// change this pose

        final Map<Integer, List<Waypoint>> RightTagIDToPosition = Map.ofEntries(
                Map.entry(
                        21,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(6.100, 4.341, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        7,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(14.536, 4.386, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        6,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(14.219, 2.828, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        17,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(3.817, 2.507, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        11,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(12.474, 2.597, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        18,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(2.915, 3.694, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        10,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(11.565, 3.679, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        19,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(3.381, 5.212, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        9,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(12.027, 5.207, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        20,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(5.109, 5.467, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        8,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(13.642, 5.481, Rotation2d.fromDegrees(-33.917)))),
                Map.entry(
                        22,
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(7.100, 8.800, Rotation2d.fromDegrees(-33.917)), // make better
                                new Pose2d(5.529, 2.778, Rotation2d.fromDegrees(-33.917)))));

        Integer aprilTagID = vision.getAprilTagID();

        // if(!vision.isTargetInView()){
        //         return new InstantCommand();
        // }

        List<Waypoint> wayPoints = new ArrayList<Waypoint>();
        if(right){
                wayPoints = RightTagIDToPosition.get(aprilTagID);
        }else{
                wayPoints = LeftTagIDToPosition.get(aprilTagID);
        }

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // test constraints(to be changed)

        PathPlannerPath path = new PathPlannerPath(
                wayPoints,
                constraints,
                null,
                new GoalEndState(0.0, Rotation2d.fromDegrees(tagIDToRotation.get(aprilTagID))));

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

    public final void zero() {
        this.tareEverything();
    }

    public void addHeadingController(double kP, double kI, double kD, double kIZone) {
        headingController = new PhoenixPIDController(kP, kI, kD);

        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(15));
    }

    public void addStrafeController(double kP, double kI, double kD) {
        strafeController = new PhoenixPIDController(kP, kI, kD);
        strafeController.setIZone(1);
        strafeController.setTolerance(0.5);
    }

    public void addForwardContrller(double kP, double kI, double kD) {
        forwardController = new PhoenixPIDController(kP, kI, kD);
        forwardController.setTolerance(1);
    }

    public void driveFieldCentricFacingAngle(double x, double y, double desiredAngle) {
        FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(x * maxSpeed)
                .withVelocityY(y * maxSpeed)
                .withTargetDirection(Rotation2d.fromDegrees(desiredAngle));
        request.HeadingController = headingController;
        request.withDeadband(0.1);
        request.withRotationalDeadband(0.04);
        this.setControl(request);
        // request.withDriveRequestType(DriveRequestType.Velocity);
    }

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
                                Math.pow(driveCont.getLeftY(), 2) *
                                        maxSpeed *
                                        -Math.signum(driveCont.getLeftY())) // Drive forward with negative Y (forward)
                        .withVelocityY(
                                Math.pow(driveCont.getLeftX(), 2) *
                                        maxSpeed *
                                        -Math.signum(driveCont.getLeftX())) // Drive left with negative X (left)
                        .withRotationalRate(
                                Math.pow(driveCont.getRightX(), 2) *
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
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    private double maxSpeed;
    private double maxAngularRate;

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
            double forwardKP,
            double forwardKI,
            double forwardKD,
            double maxSpeed,
            double maxAngularRate,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        addHeadingController(headingKP, headingKI, headingKD, headingKIZone);
        addStrafeController(stafeKP, stafeKI, stafeKD);
        addForwardContrller(forwardKP, forwardKI, forwardKD);

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

    // public boolean isFlat() {
    // double currentPitch = this.getPigeon2().getPitch().getValueAsDouble();
    // if (Math.abs(currentPitch - Constants.DRIVETRAIN_PITCH_AUTO_INIT) < 2.0 ||
    // DriverStation.isTeleop()) {
    // return true;
    // }
    // return false;
    // }

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
        Logger.recordOutput("Swerve: Current Pose", this.getPose());
        Logger.recordOutput("Swerve: Rotation", this.getRotation2d());
        Logger.recordOutput("Swerve: Angle", this.getAngle());
        // Logger.recordOutput("swerve: pithc", this.isFlat());
        Logger.recordOutput("Rotation2d", this.getPigeon2().getRotation2d());
        Logger.recordOutput(
                "Swerve: Heading Controller: Setpoint",
                headingController.getSetpoint());
        Logger.recordOutput(
                "Swerve: Heading Controller: Error",
                headingController.getPositionError());
        Logger.recordOutput(
                "Swerve: Heading Controller: AtSetpoint",
                headingController.atSetpoint());
        Logger.recordOutput(
                "Swerve: Heading Controller: PositionTolerance",
                headingController.getPositionTolerance());
        Logger.recordOutput("Swerve: CurrentState", this.getStateCopy().ModuleStates);
        Logger.recordOutput("Swerve: TargetState", this.getStateCopy().ModuleTargets);
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
    }

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
                            new PIDConstants(10, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(7, 0, 0)),
                    config,
                    // For our team, the path does not need to be flipped for Red vs Blue.
                    // The reasoning for this is that the fields are not constructed the same for
                    // each event, each side is a bit different.
                    () -> false,
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
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons());
    }
}
