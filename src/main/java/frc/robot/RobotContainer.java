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
import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.TankDrive.Drive;
import frc.robot.subsystems.TankDrive.DriveIO;
import frc.robot.subsystems.TankDrive.DriveIOSim;
import frc.robot.subsystems.TankDrive.DriveIOSpark;
import frc.robot.subsystems.TankDrive.DriveIOTalonFX;
import frc.robot.subsystems.TankDrive.DriveConstants;
import frc.robot.subsystems.imu.GyroIOPigeon2;
import frc.robot.subsystems.imu.GyroIOSim;
import frc.robot.subsystems.imu.ImuSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.FuelSim;
import frc.robot.commands.AutoScoreCommand;

/**
 * RobotContainer - The heart of the robot's organization and control structure.
 * 
 * This class is where EVERYTHING comes together:
 * - All subsystems are created here (one instance of each)
 * - All controllers (driver and operator) are defined here
 * - Button bindings connect controller buttons to commands
 * - Autonomous routines are configured and selected here
 * - Default commands are set here
 * 
 * Think of RobotContainer as the "wiring diagram" for the robot's software.
 * Just like an electrical diagram shows how components connect, this class
 * shows how subsystems, commands, and controls connect.
 * 
 * IMPORTANT: This class is created ONCE when the robot boots up.
 * The same instance is used for the entire match (and practice sessions).
 * 
 * Organization:
 * 1. Subsystem creation (with proper mode switching for sim/real/replay)
 * 2. Controller creation
 * 3. PathPlanner named command registration
 * 4. Auto chooser setup
 * 5. Button bindings
 * 6. Utility methods for autonomous
 */
public class RobotContainer {
        // ========================================
        // SUBSYSTEMS
        // ========================================
        // These are the robot's physical mechanisms.
        // Each subsystem is created ONCE and reused throughout the match.

        // Drivetrain subsystem - controls left and right motors, odometry, pathfinding
        private final Drive drive; // This field will now be initialized correctly

        // Vision subsystem - AprilTag detection for pose estimation and targeting
        private final VisionSubsystem vision = new VisionSubsystem();

        // IMU (gyroscope) - provides robot heading for odometry
        private final ImuSubsystem imu;

        // Shooter mechanism subsystems
        private final HoodSubsystem hood = new HoodSubsystem(); // Adjustable angle
        private final ShooterSubsystem shooter = new ShooterSubsystem(); // Flywheel

        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        private final IntakeRollerSubsystem intakeRollerSubsystem = new IntakeRollerSubsystem();
        public FuelSim fuelSim = new FuelSim("FuelSimTableKey"); // creates a new fuelSim of FuelSim

        // Turret subsystem - calculates auto-aim heading (virtual turret, no physical
        // rotation)
        // Declared as temporary local variable first (will be initialized after drive)
        private TurretSubsystem turret;

        // ========================================
        // DRIVER INTERFACE
        // ========================================
        // Controllers are created here and button bindings defined in
        // configureButtonBindings()

        // Driver controller (USB port 0) - controls robot movement
        private final CommandXboxController driver = new CommandXboxController(0);

        // Operator controller (USB port 1) - controls shooter, hood, and other
        // mechanisms
        private final CommandXboxController operator = new CommandXboxController(1);

        // ========================================
        // AUTONOMOUS SELECTION
        // ========================================
        // SendableChooser allows selecting autonomous routine from dashboard
        private final SendableChooser<Command> autoChooser;

        /**
         * RobotContainer constructor - called ONCE when robot boots.
         * 
         * This is where we:
         * 1. Create subsystems with appropriate IO implementations (real/sim/replay)
         * 2. Configure PathPlanner named commands
         * 3. Build autonomous chooser
         * 4. Set up button bindings
         * 5. Configure field simulation (FuelSim)
         * 
         * Order matters! Some subsystems depend on others:
         * - IMU must exist before Drive (Drive needs gyro)
         * - Drive must exist before Turret (Turret reads drive pose)
         */
        public RobotContainer() {
                // Initialize gyro simulation reference if in sim mode
                // This allows Drive and IMU to share the same simulated gyro
                GyroIOSim gyroSim = null;
                if (!RobotBase.isReal()) {
                        gyroSim = new GyroIOSim();
                }

                // Create IMU subsystem with appropriate IO implementation
                imu = new ImuSubsystem(
                                RobotBase.isReal()
                                                ? new GyroIOPigeon2(DriveConstants.pigeonCanId, "rio")
                                                : gyroSim);

                // Create Drive subsystem with mode-appropriate IO implementation
                // This switch ensures we use the right hardware interface:
                // - REAL: Actual SparkMax motor controllers
                // - SIM: Physics simulation
                // - REPLAY: No-op (just replays logged data)
                Drive tempDrive; // Use a local variable for initialization
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot - instantiate hardware IO implementations
                                // Talks to actual motor controllers via CAN bus
                                tempDrive = new Drive(new DriveIOTalonFX(), imu, vision);
                                break;

                        case SIM:
                                // Sim robot - instantiate physics sim IO implementations
                                // Simulates motor physics, encoder counts, and dynamics
                                // Uses the shared gyroSim instance for synchronized heading
                                tempDrive = new Drive(new DriveIOSim(gyroSim), imu, vision);
                                break;

                        case REPLAY: // Ensure all enum cases are handled
                        default: // Added default to cover any unhandled modes and guarantee initialization
                                 // Replayed robot - disable IO implementations
                                 // All data comes from log file instead
                                tempDrive = new Drive(new DriveIO() {
                                }, imu, vision);
                                break;
                }
                this.drive = tempDrive; // Assign the local variable to the final field

                // Basic FuelSim initialization (before turret)
                fuelSim.spawnStartingFuel();
                fuelSim.start();

                // Initialize turret after drive is assigned
                // Pass the RobotContainer's fuelSim instance
                this.turret = new TurretSubsystem(
                                vision,
                                drive,
                                shooter,
                                hood,
                                fuelSim); // Pass fuelSim here

                // NOW register robot and intake with FuelSim, as turret and drive are ready
                registerFuelSimComponents(drive, turret, intakeRollerSubsystem);

                /* ================= PATHPLANNER NAMED COMMANDS ================= */

                // Auto score: pathfind + vision dock
                NamedCommands.registerCommand(
                                "AutoScore", new AutoScoreCommand(drive, vision));

                NamedCommands.registerCommand(
                                "StopShooter",
                                shooter.stop());

                NamedCommands.registerCommand(
                                "EnableAutoAim",
                                Commands.runOnce(turret::enableHubTracking, turret));

                NamedCommands.registerCommand(
                                "DisableAutoAim",
                                Commands.runOnce(turret::disableHubTracking, turret));
                NamedCommands.registerCommand(
                                "PrepShot",
                                Commands.parallel(
                                                shooter.setRPM(1300),
                                                hood.setAngle(Degrees.of(75))));

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

                shooter.setDefaultCommand(shooter.stop());

        }

        /**
         * Configures button bindings - connects controller buttons to commands.
         * 
         * This is where you define what happens when buttons are pressed.
         * Called once from the constructor.
         * 
         * Button binding types:
         * - .whileTrue() - Command runs while button held
         * - .onTrue() - Command runs once when pressed
         * - .toggleOnTrue() - Command toggles on/off with each press
         * - .onFalse() - Triggers when button released
         * 
         * Requirements matter!
         * - Commands must addRequirements() for subsystems they use
         * - Scheduler prevents conflicts (only one command per subsystem)
         * - Default commands automatically resume when command ends
         */
        private void configureButtonBindings() {
                // ================= DRIVE DEFAULT COMMAND =================
                // This command runs continuously whenever nothing else is using drive
                drive.setDefaultCommand(
                                DriveCommands.arcadeDrive(
                                                drive,
                                                turret,
                                                () -> -driver.getLeftY(), // Forward/backward
                                                () -> -driver.getRightX())); // Rotation

                // ================= IMU / FIELD ORIENTATION =================
                // A button: Zero gyro heading (reset which way is "forward")
                // Use this if gyro drifts or you need to reset field-relative orientation
                // driver.a().onTrue(
                // Commands.runOnce(imu::zeroYaw));

                // ================= TURRET AUTO-AIM =================
                // These buttons enable/disable "heading lock" mode
                // When enabled, robot automatically rotates to face hub
                // Driver still controls forward/backward movement

                // Y button: Enable hub tracking (turn on auto-aim)
                driver.y().onTrue(
                                Commands.runOnce(turret::enableHubTracking));

                // B button: Disable hub tracking (back to manual rotation)
                driver.b().onTrue(
                                Commands.runOnce(turret::disableHubTracking));

                // ================= AUTO SCORE SEQUENCE =================
                // X button: Full autonomous scoring sequence
                // 1. Pathfind to scoring position near hub
                // 2. Vision align to exact distance
                // kCancelSelf = can be interrupted by driver taking manual control
                operator.x()
                                .onTrue(
                                                new AutoScoreCommand(drive, vision)
                                                                .withInterruptBehavior(
                                                                                Command.InterruptionBehavior.kCancelSelf));

                // ================= TURRET MANUAL ROTATION =================
                // D-pad allows manual turret adjustment (overrides auto-aim)
                // Useful for testing or if auto-aim isn't working

                // // D-pad left: Rotate turret left (negative omega)
                // operator.povLeft().whileTrue(
                // Commands.run(() -> turret.manualRotate(-0.4), turret));

                // // D-pad right: Rotate turret right (positive omega)
                // operator.povRight().whileTrue(
                // Commands.run(() -> turret.manualRotate(0.4), turret));

                // // When button released, stop manual rotation
                // operator.povLeft().onFalse(
                // Commands.runOnce(() -> turret.manualRotate(0.0)));

                // operator.povRight().onFalse(
                // Commands.runOnce(() -> turret.manualRotate(0.0)));

                // // ================= HOOD ANGLE ADJUSTMENT =================
                // // D-pad up/down for fine-tuning hood angle
                // // Adjusts in 2-degree increments

                // // D-pad up: Increase hood angle (higher arc)
                // operator.povUp().onTrue(
                // hood.setAngle(
                // hood.getAngle().plus(Degrees.of(2))));

                // // D-pad down: Decrease hood angle (flatter trajectory)
                // operator.povDown().onTrue(
                // hood.setAngle(
                // hood.getAngle().minus(Degrees.of(2))));

                // // ================= SHOOTER CONTROLS =================

                // // Right trigger: Vision-based automatic aiming
                // // Continuously adjusts shooter RPM and hood angle based on distance
                // // Deadband of 0.2 prevents accidental activation
                operator.rightTrigger(0.2).whileTrue(
                                new AimShooterFromVision(shooter, hood, vision));

                // // A button: Manual shooter test at fixed settings
                // // Useful for testing shooter mechanics without vision
                // // 1300 RPM and 75° hood angle = medium-range shot
                driver.x().whileTrue( // Driver 'X' now triggers the shoot command, which checks fuel
                                Commands.parallel(
                                                shooter.setRPM(1300),
                                                hood.setAngle(Degrees.of(75)),
                                                Commands.run(turret::shoot, turret))); // Use turret::shoot

                // Schedule `setHeight` when the Xbox controller's B button is pressed,
                // cancelling on release.
                // operator.a().whileTrue(elevatorSubsystem.setHeight(Meters.of(0.5)));
                // operator.b().whileTrue(elevatorSubsystem.setHeight(Meters.of(1)));
                driver.a().whileTrue(intakeRollerSubsystem.in(1.0)); // Driver 'A' activates intake
                // driver.y().whileTrue(intakeRollerSubsystem.out(1.0));
                // driver.b().whileTrue(intakeRollerSubsystem.stop());

        }

        /**
         * Register the robot and intake components with FuelSim.
         * This method is called after all necessary subsystems (drive, turret, intake)
         * are initialized.
         * 
         * @param drivebase       The Drive subsystem instance.
         * @param turretSubsystem The TurretSubsystem instance.
         * @param intakeSubsystem The IntakeRollerSubsystem instance.
         */
        private void registerFuelSimComponents(Drive drivebase, TurretSubsystem turretSubsystem,
                        IntakeRollerSubsystem intakeSubsystem) {
                FuelSim sim = fuelSim;

                // Register robot with real dimensions
                sim.registerRobot(
                                0.724, // width in meters
                                0.673, // length in meters
                                0.5, // bumper height (unchanged)
                                drivebase::getPose,
                                drivebase::getChassisSpeeds);

                // Define the intake bounding box in robot-centric coordinates
                double robotHalfWidth = 0.724 / 2.0;
                double robotHalfLength = 0.673 / 2.0;

                // Assuming a front-mounted intake, spanning robot width
                // Adjust these values to match your robot's actual intake geometry
                double intakeMinX = robotHalfLength - 0.1; // Extends slightly behind the front bumper
                double intakeMaxX = robotHalfLength + 0.1; // Extends slightly in front of the front bumper
                double intakeMinY = -robotHalfWidth;
                double intakeMaxY = robotHalfWidth;

                sim.registerIntake(
                                intakeMinX,
                                intakeMaxX,
                                intakeMinY,
                                intakeMaxY,
                                // shouldIntakeSupplier: only active when intakeRollerSubsystem is running AND
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
                        turretSubsystem.resetFuelStored(); // Also reset fuel count in TurretSubsystem
                }).withName("Reset Fuel").ignoringDisable(true));
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
        public Drive getDrivebase() {
                return drive;
        }

        public VisionSubsystem getVision() {
                return vision;
        }

        /**
         * Log FuelSim scoring data to AdvantageKit.
         * Call this from Robot.robotPeriodic() to continuously log scores.
         */
        public void logFuelScores() {
                Logger.recordOutput("Fuel/BlueScore", FuelSim.Hub.BLUE_HUB.getScore());
                Logger.recordOutput("Fuel/RedScore", FuelSim.Hub.RED_HUB.getScore());
        }
}