// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.TankDrive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.TankDrive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.TankDrive.DriveIO.DriveIOInputs;
import frc.robot.subsystems.imu.ImuSubsystem;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final DriveIO io;
  private final DriveIOInputs inputs = new DriveIOInputs();
  private final ImuSubsystem imu;

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);
  private final double kS = Constants.currentMode == Mode.SIM ? simKs : realKs;
  private final double kV = Constants.currentMode == Mode.SIM ? simKv : realKv;
  private final DifferentialDrivePoseEstimator poseEstimator;
  private final SysIdRoutine sysId;

  public Drive(DriveIO io, ImuSubsystem imu) {
    this.io = io;
    this.imu = imu;

    poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics,
        imu.getRotation2d(),
        0.0,
        0.0,
        Pose2d.kZero);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(
            new DifferentialDriveWheelSpeeds(
                getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec())),
        (ChassisSpeeds speeds) -> runClosedLoop(speeds),
        new PPLTVController(0.02, maxSpeedMetersPerSec),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runOpenLoop(voltage.in(Volts), voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    // Update odometry
    poseEstimator.update(imu.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());
  }

  /** Runs the drive at the desired velocity. */
  public void runClosedLoop(ChassisSpeeds speeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    runClosedLoop(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  /** Runs the drive at the desired left and right velocities. */
  public void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
    double leftRadPerSec = leftMetersPerSec / wheelRadiusMeters;
    double rightRadPerSec = rightMetersPerSec / wheelRadiusMeters;
    Logger.recordOutput("Drive/LeftSetpointRadPerSec", leftRadPerSec);
    Logger.recordOutput("Drive/RightSetpointRadPerSec", rightRadPerSec);

    double leftFFVolts = kS * Math.signum(leftRadPerSec) + kV * leftRadPerSec;
    double rightFFVolts = kS * Math.signum(rightRadPerSec) + kV * rightRadPerSec;
    io.setVelocity(leftRadPerSec, rightRadPerSec, leftFFVolts, rightFFVolts);
  }

  /** Runs the drive in open loop. */
  public void runOpenLoop(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  /** Stops the drive. */
  public void stop() {
    runOpenLoop(0.0, 0.0);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return imu.getRotation2d();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(
      imu.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters(), pose);
  }

  /** Returns the current chassis speeds. */
  @AutoLogOutput(key = "Drive/ChassisSpeeds")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
            getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec()));
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp  The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the position of the left wheels in meters. */
  @AutoLogOutput
  public double getLeftPositionMeters() {
    return inputs.leftPositionRad * wheelRadiusMeters;
  }

  /** Returns the position of the right wheels in meters. */
  @AutoLogOutput
  public double getRightPositionMeters() {
    return inputs.rightPositionRad * wheelRadiusMeters;
  }

  /** Returns the velocity of the left wheels in meters/second. */
  @AutoLogOutput
  public double getLeftVelocityMetersPerSec() {
    return inputs.leftVelocityRadPerSec * wheelRadiusMeters;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  @AutoLogOutput
  public double getRightVelocityMetersPerSec() {
    return inputs.rightVelocityRadPerSec * wheelRadiusMeters;
  }

  /** Returns the average velocity in radians/second. */
  public double getCharacterizationVelocity() {
    return (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0;
  }
}
