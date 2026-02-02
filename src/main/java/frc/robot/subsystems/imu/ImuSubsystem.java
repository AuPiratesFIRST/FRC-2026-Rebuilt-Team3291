// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** The central IMU subsystem, managing access to gyro data. */
public class ImuSubsystem extends SubsystemBase {

  private final GyroIO io;
  private final GyroIO.GyroIOInputs inputs = new GyroIO.GyroIOInputs();

  public ImuSubsystem(GyroIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IMU", inputs);
  }

  public Rotation2d getRotation2d() {
    return inputs.yawPosition;
  }

  public double getYawDeg() {
    return inputs.yawPosition.getDegrees();
  }

  public double getYawRad() {
    return inputs.yawPosition.getRadians();
  }

  public double getYawRateRadPerSec() {
    return inputs.yawVelocityRadPerSec;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public void zeroYaw() {
    io.zeroYaw();
  }
}
