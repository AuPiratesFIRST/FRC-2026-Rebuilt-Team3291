package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.PointTowardsZoneTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAndShootSmart;
import frc.robot.commands.AimShooterFromVision;
import frc.robot.commands.AutoAlignToClimb;
import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.intake.AgitatorSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.FuelSim;
import frc.robot.commands.ShooterDockAtDistanceCommand;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.PathfindThroughBalls;
import frc.robot.commands.ChaseBall;
import frc.robot.commands.PathPlannerAlign;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.VisionConstants.BLUE_TOWER_TAGS;
import static frc.robot.Constants.VisionConstants.RED_TOWER_TAGS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Lighting.LightingSubsystem;
import frc.robot.subsystems.vision.ObjectDetection;
import frc.robot.subsystems.intake.KickerSubsystem;
import frc.robot.subsystems.intake.AgitatorSubsystem;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.AutoShootOnTheMove;

// Import the YAGSL SwerveInputStream
import swervelib.SwerveInputStream;

/**
 * RobotContainer
 * -----------------------------
 * Central wiring point for the robot.
 */
public class RobotContainer {

        // ---------------- SUBSYSTEMS ----------------

        private final VisionSubsystem vision = new VisionSubsystem();

        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"), vision);
        // Shooter mechanism subsystems
        private final HoodSubsystem hood = new HoodSubsystem(); // Adjustable angle
        private final ShooterSubsystem shooter = new ShooterSubsystem(); // Flywheel
        private final Field2d m_field = new Field2d();
        private final ObjectDetection m_ballTracker = new ObjectDetection(m_field,
                        drivebase);

        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        private final IntakeRollerSubsystem intakeRollerSubsystem = new IntakeRollerSubsystem();
        private final KickerSubsystem kicker = new KickerSubsystem();
        private final AgitatorSubsystem agitatorSubsystem = new AgitatorSubsystem();
        public FuelSim fuelSim = new FuelSim("FuelSimTableKey"); // creates a new
        // fuelSim of FuelSim

        // Turret subsystem - calculates auto-aim heading (virtual turret, no physical
        // rotation)
        // Declared as temporary local variable first (will be initialized after
        // drivebase)
        private final TurretSubsystem turret = new TurretSubsystem(vision,
                        drivebase, shooter, hood, fuelSim);
        private final LightingSubsystem lighting = new LightingSubsystem();

        // ---------------- CONTROLLERS ----------------

        private final CommandXboxController driver = new CommandXboxController(0);

        private final CommandXboxController operator = new CommandXboxController(1);

        // ---------------- AUTO ----------------

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {

                // // Basic FuelSim initialization (before turret)
                fuelSim.spawnStartingFuel();
                fuelSim.start();

                // // // NOW register robot and intake with FuelSim, as turret and drive are
                // ready
                registerFuelSimComponents(drivebase, turret, intakeRollerSubsystem);
                shooter.setDefaultCommand(shooter.idle());
                intakeRollerSubsystem.setDefaultCommand(intakeRollerSubsystem.idle());
                kicker.setDefaultCommand(kicker.idle());
                agitatorSubsystem.setDefaultCommand(agitatorSubsystem.idle());
                elevatorSubsystem.setDefaultCommand(elevatorSubsystem.stow());

                /* ================= PATHPLANNER NAMED COMMANDS ================= */

                // Auto score: pathfind + vision dock
                NamedCommands.registerCommand(
                                "AutoScore", new AutoScoreCommand(drivebase, vision));

                NamedCommands.registerCommand(
                                "StopShooter",
                                shooter.stop());

                NamedCommands.registerCommand(
                                "EnableAutoAim",
                                Commands.runOnce(turret::enableHubTracking, turret));

                NamedCommands.registerCommand(
                                "DisableAutoAim",
                                Commands.runOnce(turret::disableHubTracking, turret));
                // NamedCommands.registerCommand(
                // "PrepShot",
                // Commands.parallel(
                // shooter.setRPM(1300),
                // hood.setAngle(Degrees.of(75))));
                // NamedCommands.registerCommand(
                // "ShootFixed",
                // Commands.deadline(
                // turret.shootCommand().withTimeout(0.8), // Shoot for 0.8 seconds
                // shooter.setRPM(1200),
                // hood.setAngle(Degrees.of(65))));
                NamedCommands.registerCommand("SmartShoot",
                                // 1. Aim the shooter (requires shooter/hood)
                                new AimShooterFromVision(shooter, hood, vision)

                                                // 2. Feed the ball ONLY when the shooter is at speed
                                                // This uses your intake AND your kicker subsystems simultaneously
                                                .alongWith(new AutoShootCommand(shooter, intakeRollerSubsystem, kicker,
                                                                agitatorSubsystem),
                                                                turret.shootCommand())

                                                // 3. Set a strict timeout so the robot doesn't get stuck waiting
                                                .withTimeout(9));
                NamedCommands.registerCommand("SOTM", new AutoShootOnTheMove(
                                shooter,
                                hood,
                                drivebase,
                                turret,
                                intakeRollerSubsystem,
                                kicker,
                                vision));

                NamedCommands.registerCommand("AutoIntake",
                                kicker.routeToHopper() // Command to run shooter backward/intake
                                                .alongWith(intakeRollerSubsystem.in(1.0)) // Command to run rollers at
                                                // 1.0 speed
                                                .withTimeout(1.5) // Stop both after 1.5 seconds
                                                .finallyDo(() -> { // Ensure everything stops when finished
                                                        shooter.stop();
                                                        intakeRollerSubsystem.stop();
                                                }));

                NamedCommands.registerCommand(
                                "DockAtShotDistance",
                                new ShooterDockAtDistanceCommand(
                                                vision,
                                                drivebase,
                                                3.0 // your desired shot distance meters
                                ));
                NamedCommands.registerCommand(
                                "IntakeOn",
                                intakeRollerSubsystem.in(1.0));

                NamedCommands.registerCommand(
                                "IntakeOff",
                                intakeRollerSubsystem.stop());

                // Inside RobotContainer constructor
                new PointTowardsZoneTrigger("HubArea")
                                .whileTrue(new PathPlannerAlign(turret, drivebase));

                NamedCommands.registerCommand("AutoAlign", new PathPlannerAlign(turret, drivebase).withTimeout(1));

                // NamedCommands.registerCommand("AutoAlignToClimb",
                // Commands.deferredProxy(() -> {
                // // Check the alliance color at the moment the command is called
                // boolean isRed = DriverStation.getAlliance().orElse(
                // DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

                // // Select the correct set of AprilTag IDs
                // int[] towerTags = isRed ? RED_TOWER_TAGS : BLUE_TOWER_TAGS;

                // // Build and return the final command with the correct data
                // return new AutoAlignToClimb(vision, drivebase, elevatorSubsystem, towerTags);
                // }));

                // Set up auto routines
                autoChooser = AutoBuilder.buildAutoChooser();
                autoChooser.setDefaultOption("Do Nothing", Commands.none());
                SmartDashboard.putData("Auto Chooser", autoChooser);

                configureBindings();
        }

        // --------------------------------------------------
        // CONTROLLER BINDINGS
        // --------------------------------------------------
        private void configureBindings() {

                // ================= DRIVE =================

                /**
                 * SwerveInputStream is a builder-style helper for YAGSL.
                 * It simplifies the process of:
                 * 1. Converting Controller inputs (Raw Axis) to ChassisSpeeds.
                 * 2. Applying Deadbands and Scaling in one place.
                 * 3. Handling Alliance-relative controls (field-oriented) automatically.
                 */
                // This returns 0.4 (very slow) if tracking, and 1.0 (normal) if not.
                DoubleSupplier speedMultiplier = () -> turret.isHubTrackingEnabled() ? 0.4 : 1.0;
                SwerveInputStream driveStream = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                () -> -driver.getLeftY() * speedMultiplier.getAsDouble(), // Forward/Backward
                                () -> -driver.getLeftX() * speedMultiplier.getAsDouble()) // Left/Right strafe
                                .deadband(0.1) // Apply 10% deadband to all translation axes
                                .scaleTranslation(0.7) // Reduce max speed to 80% for better control
                                .allianceRelativeControl(false) // Ensure "Forward" is always away from your alliance
                                                                // wall
                                .robotRelative(true)
                                .withControllerRotationAxis(() -> {
                                        // Simple rotation using right stick without turret integration
                                        // double rightStickX = -MathUtil.applyDeadband(driver.getRightX(), 0.1);
                                        // return rightStickX;
                                        // // Custom logic to bridge the Driver's Right Stick and the Turret Auto-Aim
                                        double rightStickX = -MathUtil.applyDeadband(driver.getRightX(), 0.1);

                                        if (Math.abs(rightStickX) > 0.05) {
                                                // Driver is actively rotating with the stick.
                                                // Call manualRotate with the stick value. This will also handle
                                                // disabling hub tracking if it was previously enabled.
                                                turret.manualRotate(rightStickX);
                                                // Directly return the stick input for immediate responsiveness.
                                                return rightStickX;
                                        } else {
                                                // Stick is in deadband (i.e., released).
                                                // If hub tracking is NOT currently active, explicitly tell the turret
                                                // to stop manual rotation by setting manualOmega to 0.0.
                                                if (!turret.isHubTrackingEnabled()) {
                                                        turret.manualRotate(0.0);
                                                }
                                                // Return the auto-aim PID output or 0.0 if disabled.
                                                return turret.getDesiredRobotOmega();
                                        }
                                });

                // Set the SwerveInputStream as the supplier for the drive command
                drivebase.setDefaultCommand(drivebase.driveCommand(driveStream));

                driver.a().onTrue(
                                Commands.runOnce(drivebase::zeroGyro));

                // Hold 'X' to automatically collect all balls in view
                // driver.x().whileTrue(new PathfindThroughBalls(drivebase, m_ballTracker));
                driver.x().whileTrue(new ChaseBall(drivebase, m_ballTracker));
                // driver.x()
                // .onTrue(
                // new AutoScoreCommand(drivebase, vision)
                // .withInterruptBehavior(
                // Command.InterruptionBehavior.kCancelSelf));

                // ================= TURRET =================
                driver.y().onTrue(
                                Commands.runOnce(turret::enableHubTracking));

                driver.povDown().whileTrue(
                                new ChaseTagCommand(
                                                vision,
                                                drivebase,
                                                new int[] { 25 },
                                                1));

                driver.b().onTrue(
                                Commands.runOnce(turret::disableHubTracking));

                operator.leftTrigger().whileTrue(intakeRollerSubsystem.in(0.5).alongWith(kicker.routeToHopper()));

                // Driver
                // 'A'
                // activates
                // intake

                // operator.povLeft().whileTrue(
                // Commands.run(() -> turret.manualRotate(-0.4), turret));

                // operator.povRight().whileTrue(
                // Commands.run(() -> turret.manualRotate(0.4), turret));

                // operator.povLeft().onFalse(
                // Commands.runOnce(() -> turret.manualRotate(0.0)));

                // operator.povRight().onFalse(
                // Commands.runOnce(() -> turret.manualRotate(0.0)));

                // // ================= HOOD =================
                // operator.povUp().onTrue(
                // hood.setAngle(
                // hood.getAngle().plus(Degrees.of(2))));

                // operator.povDown().onTrue(
                // hood.setAngle(
                // hood.getAngle().minus(Degrees.of(2))));

                // // ================= SHOOTER =================
                driver.rightTrigger(0.2).whileTrue(Commands.parallel(
                                new AimShooterFromVision(shooter, hood, vision),
                                new AutoShootCommand(shooter, intakeRollerSubsystem, kicker, agitatorSubsystem)));
                // driver.rightTrigger().whileTrue(shooter.getSysIdCommand());

                operator.rightTrigger(0.2).whileTrue(Commands.parallel(
                                new AimAndShootSmart(shooter, hood, drivebase, turret, intakeRollerSubsystem,
                                                kicker,
                                                vision, lighting)));

                // // Manual shooter test (no vision)

                operator.x().whileTrue(
                                Commands.parallel(
                                                shooter.setRPM(3000)));

                // Hold 'X' to Auto-Aim on the move. It will automatically calculate
                // the shot, compensate for drifting, and fire when locked in!

                // driver.x().whileTrue( // Operator 'X' now triggers the shoot command, which
                // checks fuel
                // Commands.parallel(
                // shooter.setRPM(1150),
                // new AutoShootCommand(shooter, intakeRollerSubsystem, kicker),
                // turret.shootCommand())); // Use turret::shoot

                // Schedule `setHeight` when the Xbox controller's B button is pressed,
                // cancelling on release.
                operator.a().whileTrue(elevatorSubsystem.setHeight(Meters.of(1)));
                operator.b().whileTrue(elevatorSubsystem.setHeight(Meters.of(-1)));
                // driver.povDown().whileTrue(
                // Commands.deferredProxy(() -> {
                // boolean isRed = DriverStation.getAlliance().orElse(
                // DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
                // int[] towerTags = isRed ? RED_TOWER_TAGS : BLUE_TOWER_TAGS;
                // return new AutoAlignToClimb(vision, drivebase, elevatorSubsystem, towerTags);
                // }));
        }

        // ================= AUTO =================
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        /**
         * Register the robot and intake components with FuelSim.
         * This method is called after all necessary subsystems (drivebase, turret,
         * intake)
         * are initialized.
         *
         * @param drivebase       The Drive subsystem instance.
         * @param turretSubsystem The TurretSubsystem instance.
         * @param intakeSubsystem The IntakeRollerSubsystem instance.
         */
        private void registerFuelSimComponents(SwerveSubsystem drivebase,
                        TurretSubsystem turretSubsystem,
                        IntakeRollerSubsystem intakeSubsystem) {
                FuelSim sim = fuelSim;

                // Register robot with real dimensions
                sim.registerRobot(
                                0.724, // width in meters
                                0.673, // length in meters
                                0.5, // bumper height (unchanged)
                                drivebase::getPose,
                                drivebase::getRobotVelocity);

                // Define the intake bounding box in robot-centric coordinates
                double robotHalfWidth = 0.724 / 2.0;
                double robotHalfLength = 0.673 / 2.0;

                // Assuming a front-mounted intake, spanning robot width
                // Adjust these values to match your robot's actual intake geometry
                double intakeMinX = robotHalfLength - 0.1; // Extends slightly behind the
                // front bumper
                double intakeMaxX = robotHalfLength + 0.1; // Extends slightly in front ofthe
                // front bumper
                double intakeMinY = -robotHalfWidth;
                double intakeMaxY = robotHalfWidth;

                sim.registerIntake(
                                intakeMinX,
                                intakeMaxX,
                                intakeMinY,
                                intakeMaxY,
                                // shouldIntakeSupplier: only active when intakeRollerSubsystem is running
                                // AND
                                // turret has capacity
                                () -> intakeSubsystem.isRunning()
                                                && turretSubsystem.getFuelStored() < TurretSubsystem.FUEL_CAPACITY,
                                () -> turretSubsystem.intakeFuel() // Callback to increment fuel count in
                // TurretSubsystem
                );

                // SmartDashboard reset button (keep here as it's FuelSim related)
                SmartDashboard.putData(Commands.runOnce(() -> {
                        sim.clearFuel();
                        sim.spawnStartingFuel();
                        turretSubsystem.resetFuelStored(); // Also reset fuel count in
                        // TurretSubsystem
                }).withName("Reset Fuel").ignoringDisable(true));
        }

        public void logFuelScores() {
                Logger.recordOutput("Fuel/BlueScore", FuelSim.Hub.BLUE_HUB.getScore());
                Logger.recordOutput("Fuel/RedScore", FuelSim.Hub.RED_HUB.getScore());
        }

        // ---------------- ACCESSORS ----------------
        public SwerveSubsystem getDrivebase() {
                return drivebase;
        }

        public VisionSubsystem getVision() {
                return vision;
        }

        public ShooterSubsystem getShooter() {
                return shooter;
        }

        public KickerSubsystem getKicker() {
                return kicker;
        }

        public ElevatorSubsystem getElevatorSubsystem() {
                return elevatorSubsystem;
        }

        public IntakeRollerSubsystem getIntakeRollerSubsystem() {
                return intakeRollerSubsystem;
        }

        public LightingSubsystem getLightingSubsystem() {
                return lighting;
        }

}