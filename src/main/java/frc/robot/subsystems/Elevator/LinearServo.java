package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;

public class LinearServo extends Servo {

    private final double m_speed; // mm per second
    private final double m_length; // mm

    private double setPos = 0.0; // target position (mm)
    private double curPos = 0.0; // estimated position (mm)

    private double lastTime;

    /**
     * Parameters for L16-R Actuonix Linear Actuators
     *
     * @param channel PWM channel used to control the servo
     * @param length  max length of the servo [mm]
     * @param speed   max speed of the servo [mm/second]
     */
    public LinearServo(int channel, double length, double speed) {
        super(9);

        // Actuonix L16 pulse width bounds in microseconds
        // Parameters: max, deadbandMax, center, deadbandMin, min
        setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        m_length = length;
        m_speed = speed;

        lastTime = Timer.getFPGATimestamp();
    }

    /**
     * Set the target position of the actuator in millimeters
     * 
     * @param setpoint target position [mm]
     */
    public void setTargetPosition(double setpoint) { // Renamed from setPosition
        setPos = MathUtil.clamp(setpoint, 0.0, m_length);

        // Servo.set() expects [0.0, 1.0]
        double servoOutput = setPos / m_length;

        // Use super.set to be explicit that we want the parent's logic
        super.set(servoOutput);
    }

    /**
     * Run this periodically to update the internal position estimate
     */
    public void updateCurPos() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        double maxDelta = m_speed * dt;

        if (curPos < setPos - maxDelta) {
            curPos += maxDelta;
        } else if (curPos > setPos + maxDelta) {
            curPos -= maxDelta;
        } else {
            curPos = setPos;
        }
    }

    /**
     * @return Estimated current position [mm]
     */
    public double getPosition() {
        return curPos;
    }

    /**
     * @return true if actuator has reached target position
     */
    public boolean isFinished() {
        return Math.abs(curPos - setPos) < 0.5; // 0.5 mm tolerance
    }

}
