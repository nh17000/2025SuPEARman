package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisualizerConstants;
import java.util.function.DoubleSupplier;
import lombok.AllArgsConstructor;
import org.littletonrobotics.junction.Logger;

@AllArgsConstructor
public class RobotVisualizer {
    private DoubleSupplier turretYawSupplier;
    private DoubleSupplier transferRollSupplier;
    private DoubleSupplier spindexerYawSupplier;
    private DoubleSupplier intakeRollSupplier;
    private DoubleSupplier hoodPitchSupplier;

    public void periodic() {
        double turretYaw = turretYawSupplier.getAsDouble();
        double transferRoll = transferRollSupplier.getAsDouble();
        double spindexerYaw = spindexerYawSupplier.getAsDouble();
        double intakeRoll = intakeRollSupplier.getAsDouble();
        double hoodPitch = hoodPitchSupplier.getAsDouble();

        Transform3d turret = new Transform3d(VisualizerConstants.M0_ZERO, new Rotation3d(0, 0, -turretYaw));
        Transform3d transfer = new Transform3d(VisualizerConstants.M1_ZERO, new Rotation3d(-transferRoll, 0, 0));
        Transform3d spindexer = new Transform3d(VisualizerConstants.M2_ZERO, new Rotation3d(0, 0, -spindexerYaw));
        Transform3d intakeS1 = new Transform3d(
                VisualizerConstants.M3_ZERO, new Rotation3d(intakeRoll - IntakeConstants.PIVOT_STARTING_ANGLE, 0, 0));
        Transform3d intakeS2 =
                new Transform3d(VisualizerConstants.M4_ZERO, new Rotation3d(getIntakeS2Angle(intakeRoll), 0, 0));
        Transform3d intakeS3 = intakeS1.plus(
                new Transform3d(VisualizerConstants.M5_OFFSET, new Rotation3d(getIntakeS3Angle(intakeRoll), 0, 0)));
        Transform3d hood = turret.plus(new Transform3d(
                VisualizerConstants.M6_OFFSET, new Rotation3d(0, -hoodPitch + HoodConstants.HOOD_STARTING_ANGLE, 0)));

        Logger.recordOutput(
                "FieldSimulation/Components2",
                new Transform3d[] {turret, transfer, spindexer, intakeS1, intakeS2, intakeS3, hood});
    }

    // https://www.desmos.com/calculator/yvnsugajwe
    private static double getIntakeS2Angle(double intakeS1Angle) {
        return 0.903162 * intakeS1Angle + 0.296121 - 1.361145254;
    }

    private static double getIntakeS3Angle(double intakeS1Angle) {
        return -1.23948 * intakeS1Angle + 0.293516 + 1.142859034;
    }
}
