// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.TankDrive;

import static frc.robot.subsystems.TankDrive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

/**
 * Real hardware implementation of DriveIO for REV Spark motor controllers.
 * 
 * This class communicates with actual Spark Max controllers over CAN bus.
 * It configures the motors, reads encoder data, and sends velocity commands.
 * 
 * HARDWARE SETUP:
 * - 4 motors total: 2 leaders + 2 followers
 * - Left leader = CAN ID from constants (leftLeaderCanId)
 * - Right leader = CAN ID from constants (rightLeaderCanId)
 * - Followers are configured to mirror their leaders
 * - Built-in encoders track wheel rotation (brushless mode)
 * 
 * KEY FEATURES:
 * - PID velocity control runs on the Spark controllers (not RoboRIO)
 * - Current limiting protects motors from burning out
 * - Voltage compensation ensures consistent behavior as battery drains
 * - Persistent configuration survives power cycles
 * 
 * ADAPTATION NOTES:
 * - For brushed motors: change MotorType.kBrushless to kBrushed
 * - For Spark Flex: replace "SparkMax" with "SparkFlex" throughout
 * - For external encoders: modify encoder setup in constructor
 */
public class DriveIOSpark implements DriveIO {
    // MOTOR CONTROLLERS
    // Leaders receive commands and control their respective side of the drivetrain
    private final SparkMax leftLeader = new SparkMax(leftLeaderCanId, MotorType.kBrushless);
    private final SparkMax rightLeader = new SparkMax(rightLeaderCanId, MotorType.kBrushless);
    // Followers mirror their leaders automatically (configured in constructor)
    private final SparkMax leftFollower = new SparkMax(leftFollowerCanId, MotorType.kBrushless);
    private final SparkMax rightFollower = new SparkMax(rightFollowerCanId, MotorType.kBrushless);
    
    // ENCODERS (built into NEO motors)
    // These track wheel rotation - our primary odometry source
    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
    
    // PID CONTROLLERS (run on Spark hardware, not RoboRIO)
    // These implement closed-loop velocity control for smooth autonomous driving
    private final SparkClosedLoopController leftController = leftLeader.getClosedLoopController();
    private final SparkClosedLoopController rightController = rightLeader.getClosedLoopController();

    public DriveIOSpark() {
        // ========================================
        // CREATE BASE CONFIGURATION
        // ========================================
        // All settings will be applied to this config object, then flashed to controllers
        var config = new SparkMaxConfig();
        
        // IDLE MODE: Brake mode makes robot stop quickly when no power applied
        // (Alternative: Coast mode lets robot drift - better for pushing matches)
        config.idleMode(IdleMode.kBrake)
              // CURRENT LIMITING: Prevents motors from drawing too much current and tripping breakers
              // Also protects motors from overheating during stalls or hard pushes
              .smartCurrentLimit(currentLimit)
              // VOLTAGE COMPENSATION: Normalizes motor behavior as battery voltage drops during match
              // Motors will automatically draw more current to maintain same speed at lower voltage
              .voltageCompensation(12.0);
        
        // PID CONFIGURATION: Tuned gains for velocity control
        // Kp = proportional gain (how hard to push toward target)
        // Ki = 0 (integral not needed for velocity control)
        // Kd = derivative gain (damping to prevent oscillation)
        config.closedLoop.pid(realKp, 0.0, realKd);
        
        // ENCODER CONFIGURATION: Unit conversions and filtering
        config.encoder
                // Convert motor shaft rotations to wheel radians
                // Motor spins faster than wheel due to gear reduction
                .positionConversionFactor(2 * Math.PI / motorReduction) // Rotor Rotations -> Wheel Radians
                // Convert motor RPM to wheel rad/s (divide by 60 to convert minutes to seconds)
                .velocityConversionFactor(
                        (2 * Math.PI) / 60.0 / motorReduction) // Rotor RPM -> Wheel Rad/Sec
                // Velocity measurement period (ms) - shorter = more responsive but noisier
                .uvwMeasurementPeriod(10)
                // Velocity averaging depth - averages last N measurements to reduce noise
                .uvwAverageDepth(2);

        // ========================================
        // APPLY CONFIGURATION TO LEADER MOTORS
        // ========================================
        // Set motor inversion for left side (may need to flip depending on mounting)
        config.inverted(leftInverted);
        // tryUntilOk() retries CAN configuration up to 5 times (CAN bus can be unreliable at startup)
        // ResetMode.kResetSafeParameters = reset everything to factory defaults first
        // PersistMode.kPersistParameters = save config to Spark flash memory (survives power cycle)
        tryUntilOk(
                leftLeader,
                5,
                () -> leftLeader.configure(
                        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        
        // Same for right side (note: one side usually needs to be inverted)
        config.inverted(rightInverted);
        tryUntilOk(
                rightLeader,
                5,
                () -> rightLeader.configure(
                        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // ========================================
        // CONFIGURE FOLLOWER MOTORS
        // ========================================
        // Followers mirror their leader's output - no separate commands needed
        // .follow() makes this motor copy another motor's output automatically
        config.inverted(leftInverted).follow(leftLeader);
        tryUntilOk(
                leftFollower,
                5,
                () -> leftFollower.configure(
                        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        config.inverted(rightInverted).follow(rightLeader);
        tryUntilOk(
                rightFollower,
                5,
                () -> rightFollower.configure(
                        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        // ========================================
        // READ LEFT SIDE SENSORS
        // ========================================
        // ifOk() handles CAN errors gracefully - if read fails, keeps old value
        // Position: Total wheel rotation since power-on (radians)
        ifOk(leftLeader, leftEncoder::getPosition, (value) -> inputs.leftPositionRad = value);
        // Velocity: Current wheel speed (rad/s)
        ifOk(leftLeader, leftEncoder::getVelocity, (value) -> inputs.leftVelocityRadPerSec = value);
        // Applied voltage: getAppliedOutput returns duty cycle (-1 to +1), multiply by bus voltage for actual volts
        ifOk(
                leftLeader,
                new DoubleSupplier[] { leftLeader::getAppliedOutput, leftLeader::getBusVoltage },
                (values) -> inputs.leftAppliedVolts = values[0] * values[1]);
        // Current draw: Read from both leader and follower motors
        ifOk(
                leftLeader,
                new DoubleSupplier[] { leftLeader::getOutputCurrent, leftFollower::getOutputCurrent },
                (values) -> inputs.leftCurrentAmps = values);

        // ========================================
        // READ RIGHT SIDE SENSORS
        // ========================================
        // Same as left side - position, velocity, voltage, and current
        ifOk(rightLeader, rightEncoder::getPosition, (value) -> inputs.rightPositionRad = value);
        ifOk(rightLeader, rightEncoder::getVelocity, (value) -> inputs.rightVelocityRadPerSec = value);
        ifOk(
                rightLeader,
                new DoubleSupplier[] { rightLeader::getAppliedOutput, rightLeader::getBusVoltage },
                (values) -> inputs.rightAppliedVolts = values[0] * values[1]);
        ifOk(
                rightLeader,
                new DoubleSupplier[] { rightLeader::getOutputCurrent, rightLeader::getOutputCurrent },
                (values) -> inputs.rightCurrentAmps = values);
    }

    /**
     * Open-loop control: Directly set motor output voltages.
     * No feedback control - motors get exactly what you command.
     */
    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        // Only leaders need commands - followers automatically mirror them
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
    }

    /**
     * Closed-loop control: Command target velocities with feedforward.
     * 
     * The Spark controllers run PID at 1kHz (much faster than RoboRIO's 50Hz)
     * to track the target velocity. Feedforward helps the PID start at the
     * right output instead of slowly ramping up.
     */
    @Override
    public void setVelocity(
            double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
        // setSetpoint() sends: target velocity + control type + PID slot + feedforward
        // ClosedLoopSlot.kSlot0 = use the PID gains we configured in constructor
        leftController.setSetpoint(
                leftRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, leftFFVolts);
        rightController.setSetpoint(
                rightRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, rightFFVolts);
    }
}
