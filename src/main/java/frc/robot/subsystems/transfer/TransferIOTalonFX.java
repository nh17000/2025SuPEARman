package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.controls.Follower;
import frc.robot.Constants.TransferConstants;
import frc.robot.util.PearadoxTalonFX;

public abstract class TransferIOTalonFX implements TransferIO {
    protected final PearadoxTalonFX leftMotor;
    protected final PearadoxTalonFX rightMotor;

    protected TransferIOTalonFX() {
        leftMotor = new PearadoxTalonFX(TransferConstants.LEFT_ID, TransferConstants.getTransferConfig());
        rightMotor = new PearadoxTalonFX(TransferConstants.RIGHT_ID, TransferConstants.getTransferConfig());

        rightMotor.setControl(new Follower(TransferConstants.LEFT_ID, true));
    }

    @Override
    public void updateInputs(TransferIOInputs inputs) {
        inputs.leftData = leftMotor.getData();
        inputs.rightData = rightMotor.getData();
    }

    @Override
    public void runVolts(double volts) {
        leftMotor.setVoltage(volts);
    }
}
