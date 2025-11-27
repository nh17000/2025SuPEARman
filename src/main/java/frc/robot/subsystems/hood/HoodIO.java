package frc.robot.subsystems.hood;

import frc.robot.util.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    static class HoodIOInputs {
        public MotorData hoodData = new MotorData();
    }

    default void updateInputs(HoodIOInputs inputs) {}

    default void runPosition(double setpointRots) {}
}
