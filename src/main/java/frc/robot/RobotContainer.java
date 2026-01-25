package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {

  // ---------------- SUBSYSTEMS ----------------
  private final VisionSubsystem vision = new VisionSubsystem();

  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(
          new File(Filesystem.getDeployDirectory(), "swerve"),
          vision
      );

  private final HoodSubsystem hood = new HoodSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem(hood);
  private final TurretSubsystem turret =
      new TurretSubsystem(
          vision,
          drivebase,
          shooter,
          hood
      );

  // ---------------- CONTROLLERS ----------------
  private final CommandXboxController driver =
      new CommandXboxController(0);

  private final CommandXboxController operator =
      new CommandXboxController(1);

  // ---------------- AUTO ----------------
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    configureBindings();

    // ---------- NamedCommands for PathPlanner ----------
    NamedCommands.registerCommand(
        "SpinUpShooter",
        Commands.runOnce(shooter::spinUp, shooter)
    );

    NamedCommands.registerCommand(
        "StopShooter",
        Commands.runOnce(shooter::stop, shooter)
    );

    NamedCommands.registerCommand(
        "EnableAutoAim",
        Commands.runOnce(turret::enableHubTracking, turret)
    );

    NamedCommands.registerCommand(
        "DisableAutoAim",
        Commands.runOnce(turret::disableHubTracking, turret)
    );

    // ---------- Auto chooser ----------
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    // ================= DRIVE =================
    drivebase.setDefaultCommand(
        drivebase.driveCommand(
            () -> -MathUtil.applyDeadband(driver.getLeftY(), 0.1),
            () -> -MathUtil.applyDeadband(driver.getLeftX(), 0.1),
            () -> {
              double stick =
                  -MathUtil.applyDeadband(driver.getRightX(), 0.1);

              // Manual override disables turret auto aim
              if (Math.abs(stick) > 0.05) {
                turret.disableHubTracking();
                turret.manualRotate(stick);
                return stick;
              }

              // Otherwise turret controls rotation
              return turret.getDesiredRobotOmega();
            }
        )
    );

    // Reset gyro
    driver.a().onTrue(
        Commands.runOnce(drivebase::zeroGyro)
    );

    // ================= TURRET AUTO AIM =================
    driver.y().onTrue(
        Commands.runOnce(turret::enableHubTracking)
    );

    driver.b().onTrue(
        Commands.runOnce(turret::disableHubTracking)
    );

    // ================= MANUAL TURRET (D-PAD) =================
    operator.povLeft().whileTrue(
        Commands.run(
            () -> turret.manualRotate(-0.4),
            turret
        )
    );

    operator.povRight().whileTrue(
        Commands.run(
            () -> turret.manualRotate(0.4),
            turret
        )
    );

    operator.povLeft().onFalse(
        Commands.runOnce(() -> turret.manualRotate(0.0))
    );

    operator.povRight().onFalse(
        Commands.runOnce(() -> turret.manualRotate(0.0))
    );

    // ================= HOOD (D-PAD) =================
    operator.povUp().onTrue(
        Commands.runOnce(
            () -> hood.setTargetAngle(
                hood.getAngle().plus(Degrees.of(2))
            )
        )
    );

    operator.povDown().onTrue(
        Commands.runOnce(
            () -> hood.setTargetAngle(
                hood.getAngle().minus(Degrees.of(2))
            )
        )
    );

    // ================= SHOOTER =================
    operator.rightTrigger().whileTrue(
        Commands.run(shooter::spinUp, shooter)
    );

    operator.rightTrigger().onFalse(
        Commands.runOnce(shooter::stop)
    );

    operator.rightBumper().whileTrue(
        Commands.run(shooter::feed, shooter)
    );

    operator.leftBumper().onTrue(
        Commands.runOnce(shooter::stop)
    );
  }

  // ================= AUTO =================
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // ---------------- ACCESSORS ----------------
  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }

  public VisionSubsystem getVision() {
    return vision;
  }
}
