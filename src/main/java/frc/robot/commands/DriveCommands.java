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

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

  private DriveCommands() {
  }

  /**
   * Arcade drive with optional turret heading lock.
   * Forward/back always from driver.
   * Rotation comes from turret when enabled.
   */
  public static Command arcadeDrive(
      Drive drive,
      TurretSubsystem turret,
      DoubleSupplier xSupplier,
      DoubleSupplier zSupplier) {

    return Commands.run(
        () -> {
          // Forward/back from driver
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);

          // Rotation source
          double omega;
          if (turret.isHubTrackingEnabled()) {
            // 🔒 Heading lock
            omega = turret.getDesiredRobotOmega();
          } else {
            // 🎮 Manual rotation
            omega = MathUtil.applyDeadband(zSupplier.getAsDouble(), DEADBAND);
          }

          var speeds = DifferentialDrive.arcadeDriveIK(x, omega, true);

          drive.runClosedLoop(
              speeds.left * maxSpeedMetersPerSec,
              speeds.right * maxSpeedMetersPerSec);
        },
        drive);
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
