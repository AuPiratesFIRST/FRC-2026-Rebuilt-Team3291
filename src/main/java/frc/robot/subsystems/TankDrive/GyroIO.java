// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.TankDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable; // Added for manual logging
import org.littletonrobotics.junction.inputs.LoggableInputs; // Added for manual logging

public interface GyroIO {
  // REMOVED: @AutoLog annotation
  // ADDED: "implements LoggableInputs"
  public static class GyroIOInputs implements LoggableInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Rotation2d.kZero;
    public double yawVelocityRadPerSec = 0.0;

    // This method tells AdvantageKit how to log the data to a table.
    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("YawPosition", yawPosition.getDegrees()); // Log as degrees for easier viewing
      table.put("YawVelocityRadPerSec", yawVelocityRadPerSec);
    }

    // This method tells AdvantageKit how to read the data back from a log for
    // replay.
    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      yawPosition = Rotation2d.fromDegrees(table.get("YawPosition", yawPosition.getDegrees()));
      yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", yawVelocityRadPerSec);
    }
  }

  public default void updateInputs(GyroIOInputs inputs) {
  }
}