package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier; // Added for the new driveCommand

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

// PathPlanner imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;
  private final VisionSubsystem vision;

  private static final double MAX_LINEAR_SPEED = 4.5; // m/s
  private static final double MAX_ANGULAR_SPEED = Math.PI * 2; // rad/s
  private double lastVisionUpdate = 0.0;
  private static final double VISION_PERIOD = 0.05; // 20 Hz

  public SwerveSubsystem(File directory, VisionSubsystem vision) {
    this.vision = vision;

    Pose2d startingPose = new Pose2d(new Translation2d(4, 4), new Rotation2d());

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive = new SwerveParser(directory)
          .createSwerveDrive(MAX_LINEAR_SPEED, startingPose);

      if (RobotBase.isSimulation()) {
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
      }
    } catch (Exception e) {
      throw new RuntimeException("Check your swerve JSON files!", e);
    }

    setupPathPlanner();
  }

  /* -------------------- TELEOP DRIVE -------------------- */

  /**
   * New driveCommand that accepts a Supplier of ChassisSpeeds.
   * This allows SwerveInputStream to be passed directly.
   */
  public Command driveCommand(Supplier<ChassisSpeeds> speedSupplier) {
    return run(() -> swerveDrive.drive(speedSupplier.get()));
  }

  public Command driveCommand(
      DoubleSupplier vX,
      DoubleSupplier vY,
      DoubleSupplier vOmega) {
    return run(() -> swerveDrive.drive(
        new Translation2d(
            vX.getAsDouble() * MAX_LINEAR_SPEED,
            vY.getAsDouble() * MAX_LINEAR_SPEED),
        vOmega.getAsDouble() * MAX_ANGULAR_SPEED,
        false,
        false));
  }

  /* -------------------- PATHPLANNER SETUP -------------------- */
  private void setupPathPlanner() {
    RobotConfig config;

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException(
          "Failed to load PathPlanner RobotConfig. Check GUI settings.",
          e);
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getRobotVelocity,
        (robotRelativeSpeeds, feedforwards) -> {
          swerveDrive.drive(
              robotRelativeSpeeds,
              swerveDrive.kinematics
                  .toSwerveModuleStates(robotRelativeSpeeds),
              feedforwards.linearForces());
        },
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)),
        config,
        () -> DriverStation.getAlliance()
            .map(a -> a == DriverStation.Alliance.Red)
            .orElse(false),
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());

    PathPlannerLogging.setLogActivePathCallback(
        path -> Logger.recordOutput(
            "Odometry/Trajectory", path.toArray(new Pose2d[0])));
  }

  /* -------------------- REQUIRED HELPERS -------------------- */

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /* -------------------- PERIODIC -------------------- */

  @Override
  public void periodic() {
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    if (now - lastVisionUpdate < VISION_PERIOD) {
      return;
    }
    lastVisionUpdate = now;

    vision.getEstimatedGlobalPose().ifPresent(est -> {
      var trust = est.targetsUsed.size() > 1
          ? VisionConstants.MULTI_TAG_STD_DEVS
          : VisionConstants.SINGLE_TAG_STD_DEVS;

      swerveDrive.addVisionMeasurement(
          est.estimatedPose.toPose2d(),
          est.timestampSeconds,
          trust);
    });
  }

  @Override
  public void simulationPeriodic() {
    vision.updateSimPose(swerveDrive.getPose());
  }

  public void drive(ChassisSpeeds speeds) {
    swerveDrive.setChassisSpeeds(speeds);
  }

  /* -------------------- ACCESSORS -------------------- */

  /**
   * Getter for the SwerveDrive object.
   * Required for RobotContainer to initialize SwerveInputStream.
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }
}