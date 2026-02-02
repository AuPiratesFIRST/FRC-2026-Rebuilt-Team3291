// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GyroIO {
  public static class GyroIOInputs implements LoggableInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Rotation2d.kZero;
    public double yawVelocityRadPerSec = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("YawDeg", yawPosition.getDegrees());
      table.put("YawRateRadPerSec", yawVelocityRadPerSec);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      yawPosition = Rotation2d.fromDegrees(table.get("YawDeg", yawPosition.getDegrees()));
      yawVelocityRadPerSec = table.get("YawRateRadPerSec", yawVelocityRadPerSec);
    }
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  /** Reset yaw to zero */
  public default void zeroYaw() {}
}
