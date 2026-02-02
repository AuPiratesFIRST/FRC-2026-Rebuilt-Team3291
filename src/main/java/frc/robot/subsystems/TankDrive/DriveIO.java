
package frc.robot.subsystems.TankDrive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DriveIO {
  // REMOVED: @AutoLog annotation
  // ADDED: "implements LoggableInputs"
  public static class DriveIOInputs implements LoggableInputs {
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {};

    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {};

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

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveIOInputs inputs) {
  }

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double leftVolts, double rightVolts) {
  }

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
  }
}