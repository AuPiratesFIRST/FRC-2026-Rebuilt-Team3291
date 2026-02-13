// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.TankDrive;

import static frc.robot.subsystems.TankDrive.DriveConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * This drive implementation is for Talon FXs driving motors like the Falon 500
 * or Kraken X60.
 * 
 * This class controls a tank drive system with 4 motors (2 per side).
 * Each side has a leader motor and a follower motor that copies the leader.
 */
public class DriveIOTalonFX implements DriveIO {
  // Motor controllers - these objects let us control the physical motors
  // Left side motors
  private final TalonFX leftLeader = new TalonFX(leftLeaderCanId);
  private final TalonFX leftFollower = new TalonFX(leftFollowerCanId);
  // Right side motors
  private final TalonFX rightLeader = new TalonFX(rightLeaderCanId);
  private final TalonFX rightFollower = new TalonFX(rightFollowerCanId);

  // Status signals - these read sensor data from the motors
  // Left side sensor readings
  private final StatusSignal<Angle> leftPosition = leftLeader.getPosition(); // How far the wheel has turned
  private final StatusSignal<AngularVelocity> leftVelocity = leftLeader.getVelocity(); // How fast the wheel is spinning
  private final StatusSignal<Voltage> leftAppliedVolts = leftLeader.getMotorVoltage(); // Voltage sent to motor
  private final StatusSignal<Current> leftLeaderCurrent = leftLeader.getSupplyCurrent(); // Electric current used
  private final StatusSignal<Current> leftFollowerCurrent = leftFollower.getSupplyCurrent();

  // Right side sensor readings
  private final StatusSignal<Angle> rightPosition = rightLeader.getPosition(); // How far the wheel has turned
  private final StatusSignal<AngularVelocity> rightVelocity = rightLeader.getVelocity(); // How fast the wheel is
                                                                                         // spinning
  private final StatusSignal<Voltage> rightAppliedVolts = rightLeader.getMotorVoltage(); // Voltage sent to motor
  private final StatusSignal<Current> rightLeaderCurrent = rightLeader.getSupplyCurrent(); // Electric current used
  private final StatusSignal<Current> rightFollowerCurrent = rightFollower.getSupplyCurrent();

  // Control request objects - reusable commands we send to motors
  private VoltageOut voltageRequest = new VoltageOut(0.0); // For direct voltage control
  private VelocityVoltage velocityRequest = new VelocityVoltage(0.0); // For speed control with feedback

  /**
   * Constructor - runs once when the robot code starts.
   * This sets up and configures all the motor controllers.
   */
  public DriveIOTalonFX() {
    // Create a configuration object with all our motor settings
    var config = new TalonFXConfiguration();
    // Set current limit to prevent motors from drawing too much power and tripping
    // breakers
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Brake mode - motors resist being turned when not powered (vs Coast mode)
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Gear ratio - converts motor rotations to wheel rotations
    config.Feedback.SensorToMechanismRatio = motorReduction;
    // PID constants - these help the motor reach target speeds accurately
    config.Slot0.kP = realKp; // Proportional gain
    config.Slot0.kD = realKd; // Derivative gain // Derivative gain

    // Set motor direction for left side (some motors need to spin backwards)
    config.MotorOutput.Inverted = leftInverted ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    // Apply the configuration to left side motors (try up to 5 times if it fails)
    tryUntilOk(5, () -> leftLeader.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> leftFollower.getConfigurator().apply(config, 0.25));

    // Set motor direction for right side
    config.MotorOutput.Inverted = rightInverted ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    // Apply the configuration to right side motors
    tryUntilOk(5, () -> rightLeader.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> rightFollower.getConfigurator().apply(config, 0.25));

    // Make follower motors copy their leader motors automatically
    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    rightFollower.setControl(new Follower(rightLeader.getDeviceID(), MotorAlignmentValue.Aligned));

    // Set how often we read sensor data (50 times per second = 50 Hz)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftLeaderCurrent,
        leftFollowerCurrent,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightLeaderCurrent,
        rightFollowerCurrent);
    // Optimize CAN bus communication to reduce network traffic
    leftLeader.optimizeBusUtilization();
    leftFollower.optimizeBusUtilization();
    rightLeader.optimizeBusUtilization();
    rightFollower.optimizeBusUtilization();
  }

  /**
   * Updates sensor readings from the motors.
   * This method is called repeatedly (usually 50 times per second) to get fresh
   * data.
   */
  @Override
  public void updateInputs(DriveIOInputs inputs) {
    // Get the latest values from all sensors at once
    BaseStatusSignal.refreshAll(
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftLeaderCurrent,
        leftFollowerCurrent,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightLeaderCurrent,
        rightFollowerCurrent);

    // Store left side data in the inputs object (converting rotations to radians)
    inputs.leftPositionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble());
    inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = new double[] { leftLeaderCurrent.getValueAsDouble(),
        leftFollowerCurrent.getValueAsDouble() };

    // Store right side data in the inputs object (converting rotations to radians)
    inputs.rightPositionRad = Units.rotationsToRadians(rightPosition.getValueAsDouble());
    inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightVelocity.getValueAsDouble());
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = new double[] {
        rightLeaderCurrent.getValueAsDouble(), rightFollowerCurrent.getValueAsDouble()
    };
  }

  /**
   * Directly controls the motors with a specified voltage.
   * Higher voltage = more power = faster spinning.
   * Used for simple control or characterization tests.
   */
  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.setControl(voltageRequest.withOutput(leftVolts));
    rightLeader.setControl(voltageRequest.withOutput(rightVolts));
  }

  /**
   * Controls the motors to spin at a target speed (velocity control).
   * The motor controller automatically adjusts voltage to maintain the speed.
   * FeedForward (FF) helps predict how much voltage is needed for smoother
   * control.
   * 
   * @param leftRadPerSec  Target speed for left side (radians per second)
   * @param rightRadPerSec Target speed for right side (radians per second)
   * @param leftFFVolts    Extra voltage for left side to help reach target faster
   * @param rightFFVolts   Extra voltage for right side to help reach target
   *                       faster
   */
  @Override
  public void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    // Send velocity command to left side (converting radians to rotations)
    leftLeader.setControl(
        velocityRequest
            .withVelocity(Units.radiansToRotations(leftRadPerSec))
            .withFeedForward(leftFFVolts));
    // Send velocity command to right side (converting radians to rotations)
    rightLeader.setControl(
        velocityRequest
            .withVelocity(Units.radiansToRotations(rightRadPerSec))
            .withFeedForward(rightFFVolts));
  }
}
