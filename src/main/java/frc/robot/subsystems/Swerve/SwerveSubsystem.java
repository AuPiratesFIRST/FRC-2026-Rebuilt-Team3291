package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
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

  /* ---------------- CONSTANTS ---------------- */

  private static final double TRACK_WIDTH = 0.61;
  private static final double MAX_LINEAR_SPEED = 3.5; // m/s
  private static final double MAX_ANGULAR_SPEED = Math.PI * 2; // rad/s

  /* ---------------- HARDWARE ---------------- */

  private final SparkMax leftLeader = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax leftFollower = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax rightLeader = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax rightFollower = new SparkMax(4, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final DifferentialDrive drive;

  /* ---------------- SENSORS ---------------- */

  private final ImuSubsystem imu = new ImuSubsystem();
  private final VisionSubsystem vision;

  /* ---------------- KINEMATICS / POSE ---------------- */

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);

  private final DifferentialDrivePoseEstimator poseEstimator;

  /* ---------------- SIM ---------------- */

  private final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),
      10.71,
      7.5,
      50,
      0.0762,
      TRACK_WIDTH,
      null);

  /* ---------------- CONSTRUCTOR (LEGACY API) ---------------- */

  public SwerveSubsystem(VisionSubsystem vision) {
    this.vision = vision;

    SparkMaxConfig leader = new SparkMaxConfig();
    leader.idleMode(SparkBaseConfig.IdleMode.kBrake);

    double metersPerRev = Math.PI * 0.1524 / 10.71;

    leader.encoder
        .positionConversionFactor(metersPerRev)
        .velocityConversionFactor(metersPerRev / 60.0);

    leftLeader.configure(leader, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    rightLeader.configure(leader, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

    leftFollower.configure(new SparkMaxConfig().follow(leftLeader),
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);

    rightFollower.configure(new SparkMaxConfig().follow(rightLeader),
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);

    rightLeader.setInverted(true);

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    drive = new DifferentialDrive(leftLeader, rightLeader);
    drive.setSafetyEnabled(false);

    poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics,
        imu.getRotation2d(),
        leftEncoder.getPosition(),
        rightEncoder.getPosition(),
        new Pose2d(),
        VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(2)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    setupPathPlanner();
  }

  /* ---------------- TELEOP DRIVE (UNCHANGED API) ---------------- */

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

  /* ---------------- CORE DRIVE (UNCHANGED API) ---------------- */

  public void drive(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheels = kinematics.toWheelSpeeds(speeds);
    wheels.desaturate(MAX_LINEAR_SPEED);

    drive.tankDrive(
        wheels.leftMetersPerSecond / MAX_LINEAR_SPEED,
        wheels.rightMetersPerSecond / MAX_LINEAR_SPEED,
        false);
  }

  /* ---------------- REQUIRED HELPERS (UNCHANGED API) ---------------- */

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
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

  public void resetOdometry(Pose2d pose) {
    imu.zeroYaw();
    poseEstimator.resetPosition(
        imu.getRotation2d(),
        leftEncoder.getPosition(),
        rightEncoder.getPosition(),
        pose);
  }

  public void zeroGyro() {
    imu.zeroYaw();
  }

  /* ---------------- PERIODIC ---------------- */

  @Override
  public void periodic() {
    poseEstimator.update(
        imu.getRotation2d(),
        leftEncoder.getPosition(),
        rightEncoder.getPosition());

    SmartDashboard.putNumber("Pose/X", getPose().getX());
    SmartDashboard.putNumber("Pose/Y", getPose().getY());
    SmartDashboard.putNumber("Pose/Heading", getPose().getRotation().getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    driveSim.setInputs(
        leftLeader.get() * 12.0,
        rightLeader.get() * 12.0);

    driveSim.update(0.02);

    leftEncoder.setPosition(driveSim.getLeftPositionMeters());
    rightEncoder.setPosition(driveSim.getRightPositionMeters());

    imu.setSimYaw(driveSim.getHeading());

    vision.updateSimPose(getPose());
  }

  /* ---------------- PATHPLANNER ---------------- */

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
        this::getRobotVelocity,
        this::drive,
        new PPLTVController(0.02),
        config,
        () -> DriverStation.getAlliance()
            .map(a -> a == DriverStation.Alliance.Red)
            .orElse(false),
        this);
  }
}
