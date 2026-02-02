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
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.VecBuilder;
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
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.TankDrive.DriveIO.DriveIOInputs;
import frc.robot.subsystems.imu.ImuSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.LocalADStarAK;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private final DriveIO io;
  private final DriveIOInputs inputs = new DriveIOInputs();
  private final ImuSubsystem imu;
  private final VisionSubsystem vision;

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);

  private final DifferentialDrivePoseEstimator poseEstimator;

  private final double kS = Constants.currentMode == Mode.SIM ? simKs : realKs;
  private final double kV = Constants.currentMode == Mode.SIM ? simKv : realKv;

  private final SysIdRoutine sysId;

  public Drive(DriveIO io, ImuSubsystem imu, VisionSubsystem vision) {
    this.io = io;
    this.imu = imu;
    this.vision = vision;

    poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics,
        imu.getRotation2d(),
        0.0,
        0.0,
        Pose2d.kZero);

    /* ---------------- PATHPLANNER ---------------- */
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        // ✅ EXPLICIT lambda removes overload ambiguity
        (speeds, feedforwards) -> runClosedLoop(speeds),
        new PPLTVController(0.02, maxSpeedMetersPerSec),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());

    PathPlannerLogging.setLogActivePathCallback(
        path -> Logger.recordOutput(
            "Odometry/Trajectory", path.toArray(new Pose2d[0])));

    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Odometry/TrajectorySetpoint", pose));

    /* ---------------- SYSID ---------------- */
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> runOpenLoop(
                volts.in(Volts),
                volts.in(Volts)),
            null,
            this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    poseEstimator.update(
        imu.getRotation2d(),
        getLeftPositionMeters(),
        getRightPositionMeters());

    vision.getEstimatedGlobalPose().ifPresent(est -> {
      var stdDevs = est.targetsUsed.size() > 1
          ? VisionConstants.MULTI_TAG_STD_DEVS
          : VisionConstants.SINGLE_TAG_STD_DEVS;

      poseEstimator.addVisionMeasurement(
          est.estimatedPose.toPose2d(),
          est.timestampSeconds,
          VecBuilder.fill(
              stdDevs.get(0, 0),
              stdDevs.get(1, 0),
              stdDevs.get(2, 0)));
    });
  }

  /* ---------------- DRIVE CONTROL ---------------- */

  /** Robot-relative closed-loop control (used by teleop + auto). */
  public void runClosedLoop(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

    runClosedLoop(
        wheelSpeeds.leftMetersPerSecond,
        wheelSpeeds.rightMetersPerSecond);
  }

  /** REQUIRED by PathPlanner — feedforwards safely ignored for tank */
  public void runClosedLoop(
      ChassisSpeeds speeds,
      DriveFeedforwards feedforwards) {
    runClosedLoop(speeds);
  }

  /** Left/right velocity control with feedforward */
  public void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
    double leftRadPerSec = leftMetersPerSec / wheelRadiusMeters;
    double rightRadPerSec = rightMetersPerSec / wheelRadiusMeters;

    Logger.recordOutput("Drive/LeftSetpointRadPerSec", leftRadPerSec);
    Logger.recordOutput("Drive/RightSetpointRadPerSec", rightRadPerSec);

    double leftFF = kS * Math.signum(leftRadPerSec) + kV * leftRadPerSec;
    double rightFF = kS * Math.signum(rightRadPerSec) + kV * rightRadPerSec;

    io.setVelocity(leftRadPerSec, rightRadPerSec, leftFF, rightFF);
  }

  public void runOpenLoop(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  public void stop() {
    runOpenLoop(0.0, 0.0);
  }

  /* ---------------- SYSID ---------------- */

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /* ---------------- ACCESSORS ---------------- */

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return imu.getRotation2d();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(
        imu.getRotation2d(),
        getLeftPositionMeters(),
        getRightPositionMeters(),
        pose);
  }

  @AutoLogOutput(key = "Drive/ChassisSpeeds")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
            getLeftVelocityMetersPerSec(),
            getRightVelocityMetersPerSec()));
  }

  /* ---------------- ENCODERS ---------------- */

  @AutoLogOutput
  public double getLeftPositionMeters() {
    return inputs.leftPositionRad * wheelRadiusMeters;
  }

  @AutoLogOutput
  public double getRightPositionMeters() {
    return inputs.rightPositionRad * wheelRadiusMeters;
  }

  @AutoLogOutput
  public double getLeftVelocityMetersPerSec() {
    return inputs.leftVelocityRadPerSec * wheelRadiusMeters;
  }

  @AutoLogOutput
  public double getRightVelocityMetersPerSec() {
    return inputs.rightVelocityRadPerSec * wheelRadiusMeters;
  }

  public double getCharacterizationVelocity() {
    return (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0;
  }
}
