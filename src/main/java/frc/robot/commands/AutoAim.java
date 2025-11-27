package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.turret.Turret;
import java.util.function.Supplier;

public class AutoAim {
    private Pose2d targetPose = FieldConstants.GOAL2.toPose2d();

    private Supplier<Pose2d> robotSupplier;

    public AutoAim(Supplier<Pose2d> robotPoseSupplier) {
        this.robotSupplier = robotPoseSupplier;
    }

    public Command aim(Turret turret, Hood hood) {
        return new RunCommand(
                () -> {
                    Pose2d robotPose = robotSupplier.get();
                    turret.followTarget(() -> getTurretTarget(robotPose, targetPose));
                    hood.followTarget(() -> getHoodTargetAngle(
                            robotPose, targetPose, FieldConstants.GOAL2.getZ() - ShooterConstants.EJECT_HEIGHT));
                },
                turret,
                hood);
    }

    private static Rotation2d getTurretTarget(Pose2d robotPose, Pose2d targetPose) {
        return robotPose
                .getRotation()
                .minus(robotPose.minus(targetPose).getTranslation().getAngle());
    }

    private static double getHoodTargetAngle(Pose2d robotPose, Pose2d targetPose, double h) {
        double x = robotPose.minus(targetPose).getTranslation().getNorm();
        double launchAngle = Math.atan(Math.abs(h / x));

        return MathUtil.clamp(Math.PI / 2 - launchAngle, HoodConstants.HOOD_MIN_ANGLE, HoodConstants.HOOD_MAX_ANGLE);
    }
}
