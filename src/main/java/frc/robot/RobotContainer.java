// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AimShooterFromVision;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.ImuSubsystem.ImuSubsystem;
import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.TankDrive.Drive;
import frc.robot.subsystems.TankDrive.DriveIO;
import frc.robot.subsystems.TankDrive.DriveIOSim;
import frc.robot.subsystems.TankDrive.DriveIOSpark;
import frc.robot.subsystems.TankDrive.GyroIO;
import frc.robot.subsystems.TankDrive.GyroIOPigeon2;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
// Import the static Units class for Degrees.of()
import static edu.wpi.first.units.Units.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Subsystems
        private final Drive drive; // This field will now be initialized correctly

        private final VisionSubsystem vision = new VisionSubsystem();
        private final ImuSubsystem imu = new ImuSubsystem(); // Unused, consider removing if not needed

        private final HoodSubsystem hood = new HoodSubsystem();
        private final ShooterSubsystem shooter = new ShooterSubsystem();

        // Declared as a temporary local variable first
        private TurretSubsystem turret;

        // ---------------- CONTROLLERS ----------------

        private final CommandXboxController driver = new CommandXboxController(0);
        private final CommandXboxController operator = new CommandXboxController(1);

        // Dashboard inputs
        private final SendableChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                Drive tempDrive; // Use a local variable for initialization
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                tempDrive = new Drive(new DriveIOSpark(), new GyroIOPigeon2());
                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                tempDrive = new Drive(new DriveIOSim(), new GyroIO() {
                                });
                                break;

                        case REPLAY: // Ensure all enum cases are handled, or a default is guaranteed
                        default: // Added default to cover any unhandled modes and guarantee initialization
                                 // Replayed robot, disable IO implementations
                                tempDrive = new Drive(new DriveIO() {
                                }, new GyroIO() {
                                });
                                break;
                }
                this.drive = tempDrive; // Assign the local variable to the final field

                // Initialize turret after drive is assigned
                this.turret = new TurretSubsystem(
                                vision,
                                drive,
                                shooter,
                                hood);

                // Set up auto routines
                autoChooser = AutoBuilder.buildAutoChooser();
                autoChooser.setDefaultOption("Do Nothing", Commands.none());
                SmartDashboard.putData("Auto Chooser", autoChooser);

                // Set up SysId routines
                autoChooser.addOption(
                                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Forward)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Reverse)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                // Default drive command, normal arcade drive
                drive.setDefaultCommand(
                                DriveCommands.arcadeDrive(
                                                drive, () -> -driver.getLeftY(), () -> -driver.getRightX()));

                // driver.a().onTrue(
                // Commands.runOnce(drive::zeroGyro)); // Assuming drive has a zeroGyro method,
                // or use imu.zeroYaw()

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

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
        // ---------------- ACCESSORS ----------------

        public VisionSubsystem getVision() {
                return vision;
        }
}