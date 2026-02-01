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

import frc.robot.commands.AimShooterFromVision;
import frc.robot.commands.ShooterDockAtDistanceCommand;
import frc.robot.subsystems.ImuSubsystem.*;
import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import static edu.wpi.first.units.Units.*;

/**
 * RobotContainer
 * -----------------------------
 * Central wiring point for the robot.
 */
public class RobotContainer {

        // ---------------- SUBSYSTEMS ----------------

        private final VisionSubsystem vision = new VisionSubsystem();
        private final ImuSubsystem imu = new ImuSubsystem();

        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"), // unused, kept for API stability
                        vision,
                        imu);

        private final HoodSubsystem hood = new HoodSubsystem();
        private final ShooterSubsystem shooter = new ShooterSubsystem();

        private final TurretSubsystem turret = new TurretSubsystem(
                        vision,
                        drivebase,
                        shooter,
                        hood);

        // ---------------- CONTROLLERS ----------------

        private final CommandXboxController driver = new CommandXboxController(0);
        private final CommandXboxController operator = new CommandXboxController(1);

        // ---------------- AUTO ----------------

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {

                configureBindings();

                // ---------------- DEFAULT COMMANDS ----------------
                hood.setDefaultCommand(hood.hold());
                shooter.setDefaultCommand(shooter.stop());

                // ---------------- PATHPLANNER NAMED COMMANDS ----------------
                NamedCommands.registerCommand(
                                "StopShooter",
                                shooter.stop());

                NamedCommands.registerCommand(
                                "EnableAutoAim",
                                Commands.runOnce(turret::enableHubTracking, turret));

                NamedCommands.registerCommand(
                                "DisableAutoAim",
                                Commands.runOnce(turret::disableHubTracking, turret));

                autoChooser = AutoBuilder.buildAutoChooser();
                autoChooser.setDefaultOption("Do Nothing", Commands.none());
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        // --------------------------------------------------
        // CONTROLLER BINDINGS
        // --------------------------------------------------
        private void configureBindings() {

                // ================= DRIVE =================
                drivebase.setDefaultCommand(
                                drivebase.driveCommand(
                                                () -> -MathUtil.applyDeadband(driver.getLeftY(), 0.1),
                                                () -> -MathUtil.applyDeadband(driver.getLeftX(), 0.1), // ignored by
                                                                                                       // diff drive
                                                () -> {
                                                        double stick = -MathUtil.applyDeadband(driver.getRightX(), 0.1);

                                                        if (Math.abs(stick) > 0.05) {
                                                                turret.disableHubTracking();
                                                                turret.manualRotate(stick);
                                                                return stick;
                                                        }

                                                        return turret.getDesiredRobotOmega();
                                                }));

                driver.a().onTrue(
                                Commands.runOnce(drivebase::zeroGyro));

                // ================= TURRET =================
                driver.y().onTrue(
                                Commands.runOnce(turret::enableHubTracking));

                driver.b().onTrue(
                                Commands.runOnce(turret::disableHubTracking));

                operator.povLeft().whileTrue(
                                Commands.run(() -> turret.manualRotate(-0.4), turret));

                operator.povRight().whileTrue(
                                Commands.run(() -> turret.manualRotate(0.4), turret));

                operator.povLeft().onFalse(
                                Commands.runOnce(() -> turret.manualRotate(0.0)));

                operator.povRight().onFalse(
                                Commands.runOnce(() -> turret.manualRotate(0.0)));

                // ================= HOOD =================
                operator.povUp().onTrue(
                                hood.setAngle(
                                                hood.getAngle().plus(Degrees.of(2))));

                operator.povDown().onTrue(
                                hood.setAngle(
                                                hood.getAngle().minus(Degrees.of(2))));

                // ================= SHOOTER =================
                operator.rightTrigger(0.2).whileTrue(
                                new AimShooterFromVision(shooter, hood, vision));

                // Manual shooter test
                operator.a().whileTrue(
                                Commands.parallel(
                                                shooter.setRPM(3000),
                                                hood.setAngle(Degrees.of(35))));
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
