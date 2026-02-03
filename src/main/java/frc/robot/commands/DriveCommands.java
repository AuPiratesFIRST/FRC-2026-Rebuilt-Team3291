package frc.robot.commands;

import static frc.robot.subsystems.TankDrive.DriveConstants.maxSpeedMetersPerSec;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.TankDrive.Drive;
import frc.robot.subsystems.Turret.TurretSubsystem;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * DriveCommands - Factory class for creating drive-related commands.
 * 
 * This class contains static methods that return Command objects for controlling
 * the drivetrain. Using factory methods keeps command creation logic organized
 * and allows reusing the same commands in multiple places.
 * 
 * Commands provided:
 * - arcadeDrive(): Main teleop drive with optional turret heading lock
 * - feedforwardCharacterization(): Measures kS and kV for tuning
 */
public class DriveCommands {
  // Joystick deadband - ignore inputs smaller than this to prevent drift
  private static final double DEADBAND = 0.1;
  
  // How fast to ramp voltage during feedforward testing (Volts per second)
  private static final double FF_RAMP_RATE = 0.1;

  // Private constructor prevents instantiation (this is a utility class)
  private DriveCommands() {
  }

  /**
   * Arcade drive command with smart turret integration.
   * 
   * This is the main teleop driving command. It reads joystick inputs and controls
   * the drivetrain. When turret auto-aim is enabled, rotation comes from the turret
   * PID controller instead of the driver's joystick.
   * 
   * Why this matters:
   * - Driver controls forward/backward speed normally
   * - When auto-aim button is pressed, robot automatically rotates to face hub
   * - Driver can still strafe/drive while auto-aim handles rotation
   * - This is called "heading lock" or "turret mode"
   * 
   * @param drive The drive subsystem to control
   * @param turret The turret subsystem (provides rotation when auto-aiming)
   * @param xSupplier Forward/backward speed (-1.0 to 1.0, typically left stick Y)
   * @param zSupplier Rotation speed (-1.0 to 1.0, typically right stick X)
   * @return Command that runs continuously (default command for drive)
   */
  public static Command arcadeDrive(
      Drive drive,
      TurretSubsystem turret,
      DoubleSupplier xSupplier,
      DoubleSupplier zSupplier) {

    return Commands.run(
        () -> {
          // Get forward/backward input from driver and apply deadband
          // Deadband prevents tiny stick drift from causing unwanted movement
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);

          // Determine rotation source: turret auto-aim OR manual driver control
          double omega;
          if (turret.isHubTrackingEnabled()) {
            // 🔒 HEADING LOCK MODE: Turret calculates rotation to aim at hub
            // Turret's PID controller calculates omega needed to face target
            omega = turret.getDesiredRobotOmega();
          } else {
            // 🎮 MANUAL MODE: Driver controls rotation with joystick
            omega = MathUtil.applyDeadband(zSupplier.getAsDouble(), DEADBAND);
          }

          // Convert arcade-style inputs to left/right wheel speeds
          // squareInputs=true makes controls less sensitive at low speeds (more precise)
          var speeds = DifferentialDrive.arcadeDriveIK(x, omega, true);

          // Command the drivetrain to run at calculated speeds
          drive.runClosedLoop(
              speeds.left * maxSpeedMetersPerSec,   // Scale to max speed
              speeds.right * maxSpeedMetersPerSec);
        },
        drive);  // Requires drive subsystem (prevents conflicts)
  }

  /** Measures the velocity feedforward constants for the drive. */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
              timer.restart();
            }),

        Commands.run(
            () -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              drive.runOpenLoop(voltage, voltage);
              velocitySamples.add(drive.getCharacterizationVelocity());
              voltageSamples.add(voltage);
            },
            drive)

            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }
}
