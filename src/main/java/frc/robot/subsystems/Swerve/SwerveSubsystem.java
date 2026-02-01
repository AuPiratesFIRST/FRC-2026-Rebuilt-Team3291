package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.ImuSubsystem.*;
import frc.robot.subsystems.vision.VisionSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import java.io.File;
import java.util.function.DoubleSupplier;

public class SwerveSubsystem extends SubsystemBase {

  /* -------------------- CONSTANTS -------------------- */

  private static final double TRACK_WIDTH_METERS = 0.61;
  private static final double MAX_LINEAR_SPEED = 3.5; // m/s
  private static final double MAX_ANGULAR_SPEED = Math.PI * 2; // rad/s

  /* -------------------- MOTORS -------------------- */

  private final SparkMax leftLeader = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax leftFollower = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax rightLeader = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax rightFollower = new SparkMax(4, MotorType.kBrushless);

  private final DifferentialDrive drive;

  /* -------------------- ENCODERS -------------------- */

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  /* -------------------- IMU / VISION -------------------- */

  private final ImuSubsystem imu;
  private final VisionSubsystem vision;

  /* -------------------- KINEMATICS / ODOMETRY -------------------- */

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

  private final DifferentialDriveOdometry odometry;

  /* -------------------- CONSTRUCTOR -------------------- */

  public SwerveSubsystem(File directory, VisionSubsystem vision, ImuSubsystem imu) {
    this.vision = vision;
    this.imu = imu;

    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

    double wheelDiameterMeters = 0.1524; // 6"
    double gearRatio = 10.71;
    double metersPerRotation = Math.PI * wheelDiameterMeters / gearRatio;

    leaderConfig.encoder
        .positionConversionFactor(metersPerRotation)
        .velocityConversionFactor(metersPerRotation / 60.0);

    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.follow(leftLeader);

    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.follow(rightLeader);

    leftLeader.configure(
        leaderConfig,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);

    rightLeader.configure(
        leaderConfig,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);

    leftFollower.configure(
        leftFollowerConfig,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);

    rightFollower.configure(
        rightFollowerConfig,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);

    rightLeader.setInverted(true);

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    drive = new DifferentialDrive(leftLeader, rightLeader);
    drive.setSafetyEnabled(false);

    odometry = new DifferentialDriveOdometry(
        imu.getRotation2d(),
        leftEncoder.getPosition(),
        rightEncoder.getPosition());

    setupPathPlanner();
  }

  /* -------------------- TELEOP -------------------- */
  /* vY intentionally ignored for differential */

  public Command driveCommand(
      DoubleSupplier vX,
      DoubleSupplier vY,
      DoubleSupplier vOmega) {

    return run(() -> drive(
        new ChassisSpeeds(
            vX.getAsDouble() * MAX_LINEAR_SPEED,
            0.0,
            vOmega.getAsDouble() * MAX_ANGULAR_SPEED)));
  }

  /* -------------------- DRIVE CORE -------------------- */

  public void drive(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

    wheelSpeeds.desaturate(MAX_LINEAR_SPEED);

    drive.tankDrive(
        wheelSpeeds.leftMetersPerSecond / MAX_LINEAR_SPEED,
        wheelSpeeds.rightMetersPerSecond / MAX_LINEAR_SPEED,
        false);
  }

  /* -------------------- PATHPLANNER -------------------- */

  private void setupPathPlanner() {
    RobotConfig config;

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getRobotVelocity, // MUST be robot-relative
        this::drive, // accepts robot-relative ChassisSpeeds
        new PPLTVController(0.02), // 20ms loop time
        config,
        () -> DriverStation.getAlliance()
            .map(a -> a == DriverStation.Alliance.Red)
            .orElse(false),
        this);
  }

  /* -------------------- HELPERS -------------------- */

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    imu.zeroYaw();

    odometry.resetPosition(
        imu.getRotation2d(),
        leftEncoder.getPosition(),
        rightEncoder.getPosition(),
        pose);
  }

  public ChassisSpeeds getRobotVelocity() {
    return kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
            leftEncoder.getVelocity(),
            rightEncoder.getVelocity()));
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getRobotVelocity(),
        imu.getRotation2d());
  }

  public void zeroGyro() {
    imu.zeroYaw();
  }

  @Override
  public void periodic() {
    odometry.update(
        imu.getRotation2d(),
        leftEncoder.getPosition(),
        rightEncoder.getPosition());

    SmartDashboard.putNumber("Pose/X", getPose().getX());
    SmartDashboard.putNumber("Pose/Y", getPose().getY());
    SmartDashboard.putNumber("Pose/Heading", getPose().getRotation().getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    vision.updateSimPose(getPose());
  }
}
