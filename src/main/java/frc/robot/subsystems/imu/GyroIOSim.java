// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

/** IO simulation implementation for the gyro. */
public class GyroIOSim implements GyroIO {
  private double simYawRad = 0.0;
  private double simYawRateRadPerSec = 0.0;
  private double lastUpdateTime = Timer.getFPGATimestamp();

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastUpdateTime;
    lastUpdateTime = currentTime;

    simYawRad += simYawRateRadPerSec * dt;

    inputs.connected = true;
    inputs.yawPosition = new Rotation2d(simYawRad);
    inputs.yawVelocityRadPerSec = simYawRateRadPerSec;
  }

  @Override
  public void zeroYaw() {
    simYawRad = 0.0;
    simYawRateRadPerSec = 0.0;
  }

  public void setSimYaw(Rotation2d yaw) {
    this.simYawRad = yaw.getRadians();
  }

  public void setSimYawRate(double rateRadPerSec) {
    this.simYawRateRadPerSec = rateRadPerSec;
  }
}
