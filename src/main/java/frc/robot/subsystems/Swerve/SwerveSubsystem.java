package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.io.File;
import java.util.function.Supplier;

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

  // Notice we removed VISION_PERIOD gating. Using `getAllUnreadResults` means
  // we want to process camera frames exactly as fast as our main robot loop runs!

  public SwerveSubsystem(File directory, VisionSubsystem vision) {
    this.vision = vision;

    Pose2d startingPose = new Pose2d(new Translation2d(3.509, 3.990), Rotation2d.fromDegrees(180));
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
    // Changed .drive() to .driveFieldOriented()
    return run(() -> swerveDrive.driveFieldOriented(speedSupplier.get()));
  }

  // public Command driveCommand(
  // DoubleSupplier vX,
  // DoubleSupplier vY,
  // DoubleSupplier vOmega) {
  // return run(() -> swerveDrive.drive(
  // new Translation2d(
  // vX.getAsDouble() * MAX_LINEAR_SPEED,
  // vY.getAsDouble() * MAX_LINEAR_SPEED),
  // vOmega.getAsDouble() * MAX_ANGULAR_SPEED,
  // false,
  // false));
  // }

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
    var poses = vision.getEstimatedGlobalPoses();

    // Loop through ALL available camera estimates from BOTH cameras and feed them
    // to YAGSL
    for (var est : poses) {

      // Dynamically choose standard deviations based on if the camera saw 1 tag or
      // multiple tags
      var trustMatrix = est.targetsUsed.size() > 1
          ? VisionConstants.MULTI_TAG_STD_DEVS
          : VisionConstants.SINGLE_TAG_STD_DEVS;

      // Add this to see the raw camera pose on the map
      Logger.recordOutput("Vision/RawCameraPose", est.estimatedPose.toPose2d());

      swerveDrive.addVisionMeasurement(
          est.estimatedPose.toPose2d(),
          est.timestampSeconds,
          trustMatrix);
    }
    SmartDashboard.putNumber("Vision/PosesCount", poses.size()); // Check if this is > 0
    swerveDrive.updateOdometry();
    vision.updateSimPose(getPose());

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