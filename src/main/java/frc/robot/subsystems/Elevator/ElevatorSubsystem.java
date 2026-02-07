package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ElevatorSubsystem extends SubsystemBase {

    // ---------------- CONFIG ----------------

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withMechanismCircumference(Meters.of(0.05))
            .withClosedLoopController(4, 0, 0,
                    MetersPerSecond.of(1),
                    MetersPerSecondPerSecond.of(2))
            .withFeedforward(new ElevatorFeedforward(0, 0, 0))
            .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE);

    // ---------------- MOTOR ----------------

    private final SparkMax motor = new SparkMax(4, MotorType.kBrushless);

    private final SmartMotorController smartMotor = new SparkWrapper(motor,
            DCMotor.getNEO(1), motorConfig);

    // ---------------- MECHANISM ----------------

    private final ElevatorConfig elevatorConfig = new ElevatorConfig(smartMotor)
            .withStartingHeight(Meters.of(0))
            .withHardLimits(Meters.of(0), Meters.of(1.5))
            .withMass(Kilograms.of(5))
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH);

    private final Elevator elevator = new Elevator(elevatorConfig);

    // ---------------- COMMAND FACTORIES ----------------

    public Command setHeight(Distance height) {
        return elevator.run(height);
    }

    public Command setHeightAndStop(Distance height) {
        return elevator.runTo(height);
    }

    public Command manual(double percent) {
        return elevator.set(percent);
    }

    public Command sysId() {
        return elevator.sysId(Volts.of(6),
                Volts.of(2).per(Seconds.of(1)),
                Seconds.of(4));
    }

    // ---------------- PERIODIC ----------------

    @Override
    public void periodic() {
        elevator.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        elevator.simIterate();
    }
}
