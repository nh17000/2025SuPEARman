package frc.robot.subsystems.shooter;

import frc.robot.util.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    static class ShooterIOInputs {
        public MotorData shooterOneData = new MotorData();
        public MotorData shooterTwoData = new MotorData();
    }

    default void updateInputs(ShooterIOInputs inputs) {}

    default void runVolts(double volts) {}
}
