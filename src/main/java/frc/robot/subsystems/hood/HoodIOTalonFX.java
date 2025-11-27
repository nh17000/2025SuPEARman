package frc.robot.subsystems.hood;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.robot.Constants.HoodConstants;
import frc.robot.util.PearadoxTalonFX;

public abstract class HoodIOTalonFX implements HoodIO {
    protected final PearadoxTalonFX hoodMotor;

    protected final MotionMagicVoltage hoodMMRequest = new MotionMagicVoltage(0);

    protected HoodIOTalonFX() {
        hoodMotor = new PearadoxTalonFX(HoodConstants.HOOD_ID, HoodConstants.getPivotConfig());
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodData = hoodMotor.getData();
    }

    @Override
    public void runPosition(double setpointRots) {
        hoodMotor.setControl(hoodMMRequest.withPosition(setpointRots));
    }
}
