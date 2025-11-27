package frc.robot.subsystems.hood;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

public class HoodIOSim extends HoodIOTalonFX {
    private final SingleJointedArmSim physicsSim = new SingleJointedArmSim(
            HoodConstants.HOOD_MOTOR,
            HoodConstants.HOOD_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(HoodConstants.HOOD_LENGTH, HoodConstants.HOOD_MASS),
            HoodConstants.HOOD_LENGTH,
            HoodConstants.HOOD_MIN_ANGLE,
            HoodConstants.HOOD_MAX_ANGLE,
            false,
            HoodConstants.HOOD_STARTING_ANGLE);

    private final TalonFXSimState hoodSimState;

    public HoodIOSim() {
        hoodSimState = hoodMotor.getSimState();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        super.updateInputs(inputs);

        hoodSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);

        physicsSim.setInputVoltage(hoodSimState.getMotorVoltage());
        physicsSim.update(Constants.LOOP_PERIOD);

        hoodSimState.setRawRotorPosition(
                (physicsSim.getAngleRads() - HoodConstants.HOOD_STARTING_ANGLE) / HoodConstants.HOOD_P_COEFFICIENT);
        hoodSimState.setRotorVelocity(physicsSim.getVelocityRadPerSec() / HoodConstants.HOOD_P_COEFFICIENT);
    }
}
