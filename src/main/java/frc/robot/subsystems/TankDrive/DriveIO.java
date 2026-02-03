
package frc.robot.subsystems.TankDrive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Hardware abstraction interface for the drivetrain.
 * 
 * This interface defines the contract for communicating with drive motors and encoders.
 * Different implementations can be swapped in for:
 * - REAL: Actual robot hardware (DriveIOSpark)
 * - SIM: Physics simulation (DriveIOSim) 
 * - REPLAY: Log playback (uses default empty methods)
 * 
 * This "IO layer" is a key part of AdvantageKit's architecture - it isolates
 * hardware-specific code so the main Drive subsystem logic works identically
 * across all three modes.
 */
public interface DriveIO {
  /**
   * Container for all sensor data read from the drivetrain hardware.
   * 
   * This class implements LoggableInputs manually (rather than using @AutoLog)
   * because it needs custom toLog() and fromLog() methods to work with
   * AdvantageKit's logging system.
   * 
   * These inputs are updated 50 times per second and automatically logged,
   * allowing us to replay matches and debug issues after the fact.
   */
  // REMOVED: @AutoLog annotation
  // ADDED: "implements LoggableInputs"
  public static class DriveIOInputs implements LoggableInputs {
    // LEFT SIDE SENSORS
    public double leftPositionRad = 0.0;          // Total rotation of left wheels (radians)
    public double leftVelocityRadPerSec = 0.0;   // Current speed of left wheels (rad/s)
    public double leftAppliedVolts = 0.0;        // Actual voltage being sent to left motors (-12 to +12V)
    public double[] leftCurrentAmps = new double[] {};  // Current draw of each left motor (amps)

    // RIGHT SIDE SENSORS
    public double rightPositionRad = 0.0;         // Total rotation of right wheels (radians)
    public double rightVelocityRadPerSec = 0.0;  // Current speed of right wheels (rad/s)
    public double rightAppliedVolts = 0.0;       // Actual voltage being sent to right motors (-12 to +12V)
    public double[] rightCurrentAmps = new double[] {}; // Current draw of each right motor (amps)

    /**
     * Writes all sensor data to AdvantageKit's log table.
     * 
     * This method is called automatically every loop cycle to save the current
     * state of all sensors. The logged data can be viewed in AdvantageScope
     * or replayed later for debugging.
     */
    // This method tells AdvantageKit how to log the data to a table.
    @Override
    public void toLog(LogTable table) {
      table.put("LeftPositionRad", leftPositionRad);
      table.put("LeftVelocityRadPerSec", leftVelocityRadPerSec);
      table.put("LeftAppliedVolts", leftAppliedVolts);
      table.put("LeftCurrentAmps", leftCurrentAmps);

      table.put("RightPositionRad", rightPositionRad);
      table.put("RightVelocityRadPerSec", rightVelocityRadPerSec);
      table.put("RightAppliedVolts", rightAppliedVolts);
      table.put("RightCurrentAmps", rightCurrentAmps);
    }

    /**
     * Reads sensor data back from a log file during replay mode.
     * 
     * In REPLAY mode, we don't have access to real hardware, so this method
     * populates the inputs with historical data from a previous match.
     * This allows us to re-run the exact same Drive logic with the exact
     * same sensor inputs to reproduce and debug issues.
     */
    // This method tells AdvantageKit how to read the data back from a log for
    // replay.
    @Override
    public void fromLog(LogTable table) {
      leftPositionRad = table.get("LeftPositionRad", leftPositionRad);
      leftVelocityRadPerSec = table.get("LeftVelocityRadPerSec", leftVelocityRadPerSec);
      leftAppliedVolts = table.get("LeftAppliedVolts", leftAppliedVolts);
      leftCurrentAmps = table.get("LeftCurrentAmps", leftCurrentAmps);

      rightPositionRad = table.get("RightPositionRad", rightPositionRad);
      rightVelocityRadPerSec = table.get("RightVelocityRadPerSec", rightVelocityRadPerSec);
      rightAppliedVolts = table.get("RightAppliedVolts", rightAppliedVolts);
      rightCurrentAmps = table.get("RightCurrentAmps", rightCurrentAmps);
    }
  }

  /**
   * Reads fresh sensor data from hardware and updates the inputs object.
   * 
   * Called every loop cycle (50Hz) before any subsystem logic runs.
   * Implementations should read encoders, current sensors, etc. and populate
   * all fields in the DriveIOInputs object.
   * 
   * Default implementation does nothing (used for REPLAY mode).
   */
  public default void updateInputs(DriveIOInputs inputs) {
  }

  /**
   * Open-loop control: directly set motor voltages.
   * 
   * Used for manual control or characterization (SysId).
   * No PID control - motors get exactly the voltage you specify.
   * 
   * @param leftVolts Voltage for left motors (-12 to +12)
   * @param rightVolts Voltage for right motors (-12 to +12)
   */
  public default void setVoltage(double leftVolts, double rightVolts) {
  }

  /**
   * Closed-loop control: command target velocities with feedforward.
   * 
   * The motor controllers will use PID to reach the target velocities.
   * Feedforward voltages (leftFFVolts, rightFFVolts) are added to help
   * the PID controller respond faster and more accurately.
   * 
   * Used for autonomous paths and smooth driving.
   * 
   * @param leftRadPerSec Target left wheel angular velocity (rad/s)
   * @param rightRadPerSec Target right wheel angular velocity (rad/s) 
   * @param leftFFVolts Feedforward voltage for left side (kS + kV*velocity)
   * @param rightFFVolts Feedforward voltage for right side (kS + kV*velocity)
   */
  public default void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
  }
}