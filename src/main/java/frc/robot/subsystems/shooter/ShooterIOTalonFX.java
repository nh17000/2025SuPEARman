package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.PearadoxTalonFX;

public abstract class ShooterIOTalonFX implements ShooterIO {
    protected final PearadoxTalonFX shooterOneMotor;
    protected final PearadoxTalonFX shooterTwoMotor;

    protected ShooterIOTalonFX() {
        shooterOneMotor = new PearadoxTalonFX(ShooterConstants.SHOOTER_ONE_ID, ShooterConstants.getShooterConfig());
        shooterTwoMotor = new PearadoxTalonFX(ShooterConstants.SHOOTER_TWO_ID, ShooterConstants.getShooterConfig());

        shooterTwoMotor.setControl(new Follower(0, true));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterOneData = shooterOneMotor.getData();
        inputs.shooterTwoData = shooterTwoMotor.getData();
    }

    @Override
    public void runVolts(double volts) {
        shooterOneMotor.setVoltage(volts);
    }
}
