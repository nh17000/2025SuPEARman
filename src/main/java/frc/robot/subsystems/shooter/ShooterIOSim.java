package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim extends ShooterIOTalonFX {
    private final DCMotorSim rollerPhysicsSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    ShooterConstants.SHOOTER_MOTORS, ShooterConstants.SHOOTER_MOI, ShooterConstants.SHOOTER_GEAR_RATIO),
            ShooterConstants.SHOOTER_MOTORS);

    private final TalonFXSimState shooterOneSimState;

    public ShooterIOSim() {
        shooterOneSimState = shooterOneMotor.getSimState();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        super.updateInputs(inputs);

        shooterOneSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);

        rollerPhysicsSim.setInputVoltage(shooterOneSimState.getMotorVoltage());
        rollerPhysicsSim.update(Constants.LOOP_PERIOD);

        shooterOneSimState.setRawRotorPosition(
                rollerPhysicsSim.getAngularPositionRad() / ShooterConstants.SHOOTER_P_COEFFICIENT);
        shooterOneSimState.setRotorVelocity(
                rollerPhysicsSim.getAngularVelocityRadPerSec() / ShooterConstants.SHOOTER_P_COEFFICIENT);
    }
}
