package frc.robot.subsystems.ImuSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class ImuSubsystem extends SubsystemBase {

    private final Pigeon2 pigeon;
    private final Pigeon2SimState simState;

    public ImuSubsystem() {
        pigeon = new Pigeon2(17, "rio");

        Pigeon2Configuration config = new Pigeon2Configuration();
        pigeon.getConfigurator().apply(config);
        pigeon.clearStickyFaults();

        simState = RobotBase.isSimulation() ? pigeon.getSimState() : null;
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(pigeon.getYaw().getValue());
    }

    public double getYawDeg() {
        return pigeon.getYaw().getValueAsDouble();
    }

    public void zeroYaw() {
        pigeon.setYaw(0);
    }

    /* ---------------- SIM ONLY ---------------- */

    public void setSimYaw(Rotation2d yaw) {
        if (RobotBase.isSimulation() && simState != null) {
            simState.setRawYaw(yaw.getDegrees());
        }
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            BaseStatusSignal.refreshAll(
                    pigeon.getYaw(),
                    pigeon.getPitch(),
                    pigeon.getRoll());
        }

        Logger.recordOutput("IMU/YawDeg", getYawDeg());
    }
}
