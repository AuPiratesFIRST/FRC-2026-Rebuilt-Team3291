package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.LinearVelocity;

import org.littletonrobotics.junction.Logger;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.*;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase {

    private static final double MAX_RPM = 6000.0;

    private final SparkMax shooterMotor = new SparkMax(28, MotorType.kBrushless);

    private final SmartMotorController shooterSMC = new SparkWrapper(
            shooterMotor,
            DCMotor.getNEO(1),
            new SmartMotorControllerConfig(this)
                    .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
                    .withClosedLoopController(
                            0.001, 0.0, 0.0,
                            RPM.of(MAX_RPM),
                            RotationsPerSecondPerSecond.of(1200))
                    .withFeedforward(
                            new SimpleMotorFeedforward(0.25, 0.12, 0.015))
                    .withGearing(
                            new MechanismGearing(
                                    GearBox.fromReductionStages(3, 4)))
                    .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withTelemetry("ShooterMotor",
                            SmartMotorControllerConfig.TelemetryVerbosity.MID));

    private final FlyWheel flywheel = new FlyWheel(
            new FlyWheelConfig(shooterSMC)
                    .withDiameter(Inches.of(4))
                    .withMass(Pounds.of(1))
                    .withUpperSoftLimit(RPM.of(MAX_RPM))
                    .withTelemetry("ShooterMech",
                            SmartMotorControllerConfig.TelemetryVerbosity.LOW));

    private double lastTargetRPM = 0.0;

    // ------------------------------------------------
    // COMMAND FACTORIES
    // ------------------------------------------------

    public Command setRPM(double rpm) {
        return flywheel.setSpeed(() -> {
            lastTargetRPM = rpm;
            return RPM.of(rpm);
        });
    }

    public Command stop() {
        return setRPM(0);
    }

    // ------------------------------------------------
    // DIRECT APPLY (USED BY COMMANDS)
    // ------------------------------------------------
    public LinearVelocity getExitVelocity() {
        // Flywheel diameter = 4 inches
        double diameterMeters = Inches.of(4).in(Meters);
        double circumference = Math.PI * diameterMeters;

        // RPM → rotations per second
        double rps = flywheel.getSpeed().in(RPM) / 60.0;

        // v = rps × circumference
        return MetersPerSecond.of(rps * circumference);
    }

    public void applyRPM(double rpm) {
        lastTargetRPM = rpm;
        flywheel.setSpeed(RPM.of(rpm)).schedule();
    }

    public double getTargetRPM() {
        return lastTargetRPM;
    }

    public double getActualRPM() {
        return flywheel.getSpeed().in(RPM);
    }

    // ------------------------------------------------
    // PERIODIC
    // ------------------------------------------------

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/TargetRPM", lastTargetRPM);
        Logger.recordOutput("Shooter/ActualRPM",
                flywheel.getSpeed().in(RPM));

        flywheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }
}
