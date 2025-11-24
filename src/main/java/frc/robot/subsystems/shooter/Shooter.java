package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    @AutoLogOutput
    @Getter
    @Setter
    private ShooterState state = ShooterState.FULL;

    private ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        io.runVolts(state.volts);
    }

    @AutoLogOutput
    public double getAngularPositionRads() {
        return inputs.shooterOneData.position() * ShooterConstants.SHOOTER_P_COEFFICIENT;
    }
}
