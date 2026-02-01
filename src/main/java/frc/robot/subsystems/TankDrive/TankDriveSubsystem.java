package frc.robot.subsystems.TankDrive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class TankDriveSubsystem extends SubsystemBase {

  /* ---------------- CONSTANTS ---------------- */

  private static final double TRACK_WIDTH = 0.61;
  private static final double MAX_LINEAR_SPEED = 3.5;
  /** Max angular speed (rad/s) for scaling manual rotation stick input. */
  public static final double MAX_ANGULAR_SPEED = 3.0;

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

  /* ---------------- POSE ---------------- */

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);

  private final DifferentialDrivePoseEstimator poseEstimator;

  /* ---------------- FIELD ---------------- */

  private final Field2d field = new Field2d();

  /* ---------------- SIM ---------------- */

  private final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),
      10.71,
      7.5,
      50,
      0.0762,
      TRACK_WIDTH,
      null);

  /* ---------------- PATHPLANNER ---------------- */

  private PathPlannerTrajectory activeTrajectory;
  private double trajectoryStartTime = 0.0;

  /* ---------------- CONSTRUCTOR ---------------- */

  public TankDriveSubsystem(VisionSubsystem vision) {
    this.vision = vision;

    SparkMaxConfig leader = new SparkMaxConfig();
    leader.idleMode(SparkBaseConfig.IdleMode.kBrake);

    double metersPerRev = Math.PI * 0.1524 / 10.71;

    leader.encoder
        .positionConversionFactor(metersPerRev)
        .velocityConversionFactor(metersPerRev / 60.0);

    leftLeader.configure(
        leader,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);

    rightLeader.configure(
        leader,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);

    leftFollower.configure(
        new SparkMaxConfig().follow(leftLeader),
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);

    rightFollower.configure(
        new SparkMaxConfig().follow(rightLeader),
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

    SmartDashboard.putData("Field", field);

    setupPathPlanner();
  }

  /* ---------------- DRIVE ---------------- */

  public void drive(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheels = kinematics.toWheelSpeeds(speeds);
    wheels.desaturate(MAX_LINEAR_SPEED);

    drive.tankDrive(
        wheels.leftMetersPerSecond / MAX_LINEAR_SPEED,
        wheels.rightMetersPerSecond / MAX_LINEAR_SPEED,
        false);

    Logger.recordOutput("Drive/TargetSpeeds", speeds);
  }

  /**
   * Returns a Command that drives the robot from supplier inputs.
   * Forward and strafe are -1 to 1 (strafe ignored for tank drive). Rotation is rad/s.
   */
  public Command driveCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
    return Commands.run(
        () -> drive(new ChassisSpeeds(
            forward.getAsDouble() * MAX_LINEAR_SPEED,
            strafe.getAsDouble() * MAX_LINEAR_SPEED,
            rotation.getAsDouble())),
        this);
  }

  /* ---------------- ACCESSORS ---------------- */

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public ChassisSpeeds getRobotVelocity() {
    return kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
            leftEncoder.getVelocity(),
            rightEncoder.getVelocity()));
  }

  public void resetOdometry(Pose2d pose) {
    imu.zeroYaw();
    poseEstimator.resetPosition(
        imu.getRotation2d(),
        leftEncoder.getPosition(),
        rightEncoder.getPosition(),
        pose);
  }

  /* ---------------- PERIODIC ---------------- */

  @Override
  public void periodic() {
    poseEstimator.update(
        imu.getRotation2d(),
        leftEncoder.getPosition(),
        rightEncoder.getPosition());

    Pose2d pose2d = getPose();
    field.setRobotPose(pose2d);

    Logger.recordOutput("Drive/Pose2d", pose2d);
    Logger.recordOutput(
        "Drive/WheelSpeeds",
        new DifferentialDriveWheelSpeeds(
            leftEncoder.getVelocity(),
            rightEncoder.getVelocity()));

    if (activeTrajectory != null) {
      double t = Timer.getFPGATimestamp() - trajectoryStartTime;

      if (t <= activeTrajectory.getTotalTimeSeconds()) {
        var sample = activeTrajectory.sample(t);

        Logger.recordOutput(
            "Drive/Trajectory/TargetPose",
            sample.pose);
      }
    }
  }

  /* ---------------- SIM ---------------- */

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

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getRobotVelocity(),
        imu.getRotation2d());
  }

  public void zeroGyro() {
    imu.zeroYaw();
  }

  public void setActiveTrajectory(PathPlannerTrajectory trajectory) {
    this.activeTrajectory = trajectory;
    trajectoryStartTime = Timer.getFPGATimestamp();

    var obj = field.getObject("Trajectory");
    obj.getPoses().clear();

    for (double t = 0; t <= trajectory.getTotalTimeSeconds(); t += 0.1) {
      obj.getPoses().add(trajectory.sample(t).pose);
    }
  }
}
