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

/**
 * Drive Subsystem - Controls the robot's tank drivetrain.
 * 
 * This subsystem manages both left and right motor groups, tracks the robot's position
 * on the field using odometry (wheel encoders + gyro + vision), and provides methods
 * for both driver control and autonomous path following.
 * 
 * Key Features:
 * - Hardware abstraction through DriveIO interface (works with real robot, sim, and replay)
 * - Pose estimation using wheel encoders, IMU heading, and vision AprilTag corrections
 * - PathPlanner integration for autonomous navigation
 * - AdvantageKit logging for all sensor data
 * - SysId support for characterizing motor feedforward constants
 */
public class Drive extends SubsystemBase {

  // ========================================
  // HARDWARE INTERFACES
  // ========================================
  // DriveIO is the hardware abstraction layer - it talks to motors and encoders.
  // This allows the same code to work with real hardware, simulation, or log replay.
  private final DriveIO io;
  
  // DriveIOInputs stores all sensor readings (encoder positions, velocities, currents, etc.)
  // These are updated every loop cycle (50Hz) and automatically logged by AdvantageKit
  private final DriveIOInputs inputs = new DriveIOInputs();
  
  // IMU (gyroscope) provides the robot's heading angle - critical for knowing which way we're facing
  private final ImuSubsystem imu;
  
  // Vision subsystem detects AprilTags to correct accumulated odometry drift
  private final VisionSubsystem vision;

  // ========================================
  // KINEMATICS & ODOMETRY
  // ========================================
  // Kinematics converts between:
  // - Robot chassis speeds (forward velocity + rotation rate)
  // - Individual wheel speeds (left wheel speed + right wheel speed)
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);

  // Pose Estimator is the "brain" that fuses multiple data sources to estimate where we are:
  // 1. Wheel encoder odometry (primary, runs every loop)
  // 2. Gyro heading (corrects rotation drift)
  // 3. Vision measurements (corrects XY position drift when we see AprilTags)
  private final DifferentialDrivePoseEstimator poseEstimator;

  // ========================================
  // FEEDFORWARD CONSTANTS
  // ========================================
  // These constants characterize how the motors respond to voltage
  // kS = Static friction (voltage needed to overcome friction and start moving)
  // kV = Velocity gain (additional voltage needed per unit of velocity)
  // We use different values for sim vs real robot because physics simulation isn't perfect
  private final double kS = Constants.currentMode == Mode.SIM ? simKs : realKs;
  private final double kV = Constants.currentMode == Mode.SIM ? simKv : realKv;

  // ========================================
  // SYSTEM IDENTIFICATION
  // ========================================
  // SysId is WPILib's tool for automatically measuring kS and kV
  // It runs test routines that slowly ramp up voltage and measure the response
  private final SysIdRoutine sysId;

  /**
   * Creates a new Drive subsystem.
   * 
   * @param io The hardware interface (real motors, simulation, or replay)
   * @param imu The gyroscope subsystem for heading measurements
   * @param vision The vision subsystem for AprilTag pose corrections
   */
  public Drive(DriveIO io, ImuSubsystem imu, VisionSubsystem vision) {
    this.io = io;
    this.imu = imu;
    this.vision = vision;

    // Initialize the pose estimator at the origin (0, 0, 0 degrees)
    // This will be updated to the actual starting position before the match
    poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics,           // Our kinematics model
        imu.getRotation2d(),  // Initial heading from gyro
        0.0,                  // Initial left wheel position
        0.0,                  // Initial right wheel position
        Pose2d.kZero);        // Starting pose (origin)

    /* ---------------- PATHPLANNER CONFIGURATION ---------------- */
    // PathPlanner is our autonomous path following library
    // AutoBuilder.configure() tells PathPlanner how to control our robot
    AutoBuilder.configure(
        this::getPose,              // How to get current position
        this::setPose,              // How to reset position
        this::getChassisSpeeds,     // How to get current velocity
        // How to drive the robot (the actual control method)
        // Note: PathPlanner passes feedforwards but we calculate our own internally
        (speeds, feedforwards) -> runClosedLoop(speeds),
        // Controller: PPLTVController = "Linear Time-Varying" controller
        // 0.02 = update period (20ms), maxSpeedMetersPerSec = speed limit
        new PPLTVController(0.02, maxSpeedMetersPerSec),
        ppConfig,                   // Robot config (track width, max speeds, etc.)
        // Should we flip paths? (Red alliance mirrors Blue alliance)
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);                      // This subsystem

    // Set up A* pathfinding algorithm for generating paths on-the-fly
    // LocalADStarAK is a custom AdvantageKit-compatible version of the AD* algorithm
    Pathfinding.setPathfinder(new LocalADStarAK());

    // Hook up PathPlanner to AdvantageKit logging
    // This logs the planned path trajectory so we can visualize it in AdvantageScope
    PathPlannerLogging.setLogActivePathCallback(
        path -> Logger.recordOutput(
            "Odometry/Trajectory", path.toArray(new Pose2d[0])));

    // Also log where PathPlanner wants us to be at this moment
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Odometry/TrajectorySetpoint", pose));

    /* ---------------- SYSID CHARACTERIZATION ---------------- */
    // SysId helps us measure kS and kV by running automated tests
    // It applies gradually increasing voltages and measures the response
    sysId = new SysIdRoutine(
        // Config: use defaults for ramp rate, step voltage, and timeout
        // We only customize the logging callback
        new SysIdRoutine.Config(
            null,  // Use default ramp rate
            null,  // Use default step voltage
            null,  // Use default timeout
            state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        // Mechanism: defines how to apply voltage and what to measure
        new SysIdRoutine.Mechanism(
            // How to drive motors at a specific voltage
            volts -> runOpenLoop(
                volts.in(Volts),  // Apply same voltage to both sides
                volts.in(Volts)),
            null,  // No special log consumer
            this));
  }

  /**
   * Called automatically every 20ms by the CommandScheduler.
   * Updates odometry, fuses vision measurements, and logs everything.
   */
  @Override
  public void periodic() {
    // STEP 1: Read fresh sensor data from hardware
    // This updates encoder positions, velocities, motor currents, etc.
    io.updateInputs(inputs);
    
    // STEP 2: Send all sensor data to AdvantageKit for logging
    // This creates timestamped logs we can replay later or view in AdvantageScope
    Logger.processInputs("Drive", inputs);

    // STEP 3: Update odometry using wheel encoders and gyro
    // This is the PRIMARY pose update that runs every loop (50 Hz)
    // It tracks how far the wheels have turned since last loop
    poseEstimator.update(
        imu.getRotation2d(),          // Current heading from gyro
        getLeftPositionMeters(),      // Left wheel distance traveled
        getRightPositionMeters());    // Right wheel distance traveled

    // STEP 4: Fuse vision measurements when available
    // Vision corrects accumulated drift from wheel slippage
    // .ifPresent() only runs if we currently see AprilTags
    vision.getEstimatedGlobalPose().ifPresent(est -> {
      // Choose standard deviations based on number of tags seen:
      // - Multi-tag (2+): More accurate, so we trust it more (smaller std dev)
      // - Single-tag: Less accurate, trust it less (larger std dev)
      // Standard deviation tells the pose estimator how much to trust this measurement
      var stdDevs = est.targetsUsed.size() > 1
          ? VisionConstants.MULTI_TAG_STD_DEVS   // [0.5, 0.5, 0.9] (example values)
          : VisionConstants.SINGLE_TAG_STD_DEVS; // [1.0, 1.0, 2.0] (example values)

      // Add vision measurement with its timestamp
      // Using timestamp (not current time) accounts for camera processing latency
      poseEstimator.addVisionMeasurement(
          est.estimatedPose.toPose2d(),  // 2D pose from vision
          est.timestampSeconds,           // When image was captured
          VecBuilder.fill(
              stdDevs.get(0, 0),  // X position std dev (meters)
              stdDevs.get(1, 0),  // Y position std dev (meters)
              stdDevs.get(2, 0))); // Rotation std dev (radians)
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

  /**
   * Open-loop voltage control (used by SysId characterization).
   * Directly applies voltages to motors without any velocity feedback control.
   * 
   * @param leftVolts Voltage for left motors (-12 to +12)
   * @param rightVolts Voltage for right motors (-12 to +12)
   */
  public void runOpenLoop(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  /**
   * Stops the drivetrain by setting motor voltages to zero.
   */
  public void stop() {
    runOpenLoop(0.0, 0.0);
  }

  /* ---------------- SYSID CHARACTERIZATION COMMANDS ---------------- */
  // These commands run automated tests to measure kS and kV
  // To use: Select these commands in the auto chooser, enable, and watch the console

  /**
   * Quasistatic test - slowly ramps voltage up and measures steady-state velocity.
   * This helps identify kV (the voltage-to-velocity ratio).
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /**
   * Dynamic test - quickly steps voltage and measures acceleration.
   * This helps identify kS (static friction) and system inertia.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /* ---------------- ODOMETRY ACCESSORS ---------------- */
  // These methods are used by commands, PathPlanner, and logging

  /**
   * Returns the robot's estimated pose on the field.
   * This is the result of fusing wheel odometry, gyro, and vision measurements.
   * 
   * @return Current pose (x, y in meters, heading in Rotation2d)
   * @AutoLogOutput automatically logs this to "Odometry/Robot" key
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns the robot's current heading from the gyro.
   * 
   * @return Current rotation (0 degrees = facing away from driver station)
   */
  public Rotation2d getRotation() {
    return imu.getRotation2d();
  }

  /**
   * Resets the robot's pose to a specific position.
   * Used at the start of autonomous to set our known starting position.
   * 
   * CAUTION: Only call this when you're CERTAIN of the robot's position!
   * Incorrect resets will mess up autonomous navigation.
   * 
   * @param pose The pose to reset to (typically from PathPlanner or alliance station)
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(
        imu.getRotation2d(),          // Current gyro heading
        getLeftPositionMeters(),      // Current left encoder reading
        getRightPositionMeters(),     // Current right encoder reading
        pose);                        // New pose we're claiming to be at
  }

  /**
   * Returns the robot's current velocity in chassis-relative coordinates.
   * Used by PathPlanner to know how fast we're currently moving.
   * 
   * @return ChassisSpeeds (vx, vy, omega) - vy is always 0 for tank drive
   * @AutoLogOutput automatically logs this to "Drive/ChassisSpeeds" key
   */
  @AutoLogOutput(key = "Drive/ChassisSpeeds")
  public ChassisSpeeds getChassisSpeeds() {
    // Convert individual wheel speeds back to robot speeds
    return kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
            getLeftVelocityMetersPerSec(),
            getRightVelocityMetersPerSec()));
  }

  /* ---------------- ENCODER ACCESSORS ---------------- */
  // These methods convert raw encoder readings (radians) to linear measurements (meters)

  /**
   * Returns total distance the left wheels have traveled.
   * Converts from encoder radians to linear meters using wheel radius.
   * 
   * @return Left wheel position in meters
   */
  @AutoLogOutput
  public double getLeftPositionMeters() {
    return inputs.leftPositionRad * wheelRadiusMeters;
  }

  /**
   * Returns total distance the right wheels have traveled.
   * 
   * @return Right wheel position in meters
   */
  @AutoLogOutput
  public double getRightPositionMeters() {
    return inputs.rightPositionRad * wheelRadiusMeters;
  }

  /**
   * Returns current velocity of left wheels.
   * Converts from encoder rad/s to linear m/s.
   * 
   * @return Left wheel velocity in meters per second
   */
  @AutoLogOutput
  public double getLeftVelocityMetersPerSec() {
    return inputs.leftVelocityRadPerSec * wheelRadiusMeters;
  }

  /**
   * Returns current velocity of right wheels.
   * 
   * @return Right wheel velocity in meters per second
   */
  @AutoLogOutput
  public double getRightVelocityMetersPerSec() {
    return inputs.rightVelocityRadPerSec * wheelRadiusMeters;
  }

  /**
   * Returns average velocity of both sides (used by SysId characterization).
   * 
   * @return Average angular velocity in radians per second
   */
  public double getCharacterizationVelocity() {
    return (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0;
  }
}
