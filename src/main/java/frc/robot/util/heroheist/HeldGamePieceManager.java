package frc.robot.util.heroheist;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.Constants.VisualizerConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

public class HeldGamePieceManager {
    private HeldSpeechBubble[] spindexerSlots = new HeldSpeechBubble[SpindexerConstants.SPINDEXER_CAPACITY];

    private List<HeldSpeechBubble> bubbles = new ArrayList<>();

    private DoubleSupplier intakeVelocitySupplier;
    private DoubleSupplier spindexerPositionSupplier;
    private DoubleSupplier transferVelocitySupplier;
    private DoubleSupplier shooterVelocitySupplier;
    private DoubleSupplier hoodAngleSupplier;
    private DoubleSupplier turretAngleSupplier;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private final IntakeSimulation bubbleIntakeSim;

    public HeldGamePieceManager(
            DoubleSupplier intakeVelocitySupplier,
            DoubleSupplier spindexerPositionSupplier,
            DoubleSupplier transferVelocitySupplier,
            DoubleSupplier shooterVelocitySupplier,
            DoubleSupplier hoodAngleSupplier,
            DoubleSupplier turretAngleSupplier,
            AbstractDriveTrainSimulation driveSimulation) {

        this.intakeVelocitySupplier = intakeVelocitySupplier;
        this.spindexerPositionSupplier = spindexerPositionSupplier;
        this.transferVelocitySupplier = transferVelocitySupplier;
        this.shooterVelocitySupplier = shooterVelocitySupplier;
        this.hoodAngleSupplier = hoodAngleSupplier;
        this.turretAngleSupplier = turretAngleSupplier;
        this.poseSupplier = driveSimulation::getSimulatedDriveTrainPose;
        this.chassisSpeedsSupplier = driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative;

        bubbleIntakeSim = IntakeSimulation.OverTheBumperIntake(
                "Speech Bubble", driveSimulation, Inches.of(20), Inches.of(3), IntakeSimulation.IntakeSide.LEFT, 6);
    }

    public void periodic() {
        if (intakeVelocitySupplier.getAsDouble() > 0.1) {
            bubbleIntakeSim.startIntake();
        } else {
            bubbleIntakeSim.stopIntake();
        }

        if (bubbleIntakeSim.obtainGamePieceFromIntake()) {
            bubbles.add(new HeldSpeechBubble());
        }

        double intakeVel = intakeVelocitySupplier.getAsDouble() * IntakeConstants.ROLLER_RADIUS;
        double spindexerPos = spindexerPositionSupplier.getAsDouble();
        double transferVel = transferVelocitySupplier.getAsDouble() * TransferConstants.TRANSFER_RADIUS;
        double shooterVel = shooterVelocitySupplier.getAsDouble() * ShooterConstants.SHOOTER_RADIUS;
        Pose2d robotPose = poseSupplier.get();

        var iterator = bubbles.iterator();
        List<Pose3d> bubblePoses = new ArrayList<>();
        while (iterator.hasNext()) {
            HeldSpeechBubble b = iterator.next();
            boolean shouldEject = b.update(Constants.LOOP_PERIOD, intakeVel, spindexerPos, transferVel, shooterVel);
            if (shouldEject) {
                iterator.remove();
                launch(shooterVel);
            } else {
                bubblePoses.add(new Pose3d(robotPose).plus(b.transform));
            }
        }

        Logger.recordOutput("FieldSimulation/Held Bubbles", bubblePoses.toArray(new Pose3d[bubblePoses.size()]));
    }

    private void launch(double shooterVel) {
        Transform3d shooterTransform = new Transform3d(
                VisualizerConstants.M6_ZERO, new Rotation3d(0, Math.PI / 2 - hoodAngleSupplier.getAsDouble(), 0));

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new SpeechBubbleOnFly(
                        poseSupplier.get().getTranslation(),
                        shooterTransform.getTranslation().toTranslation2d(),
                        chassisSpeedsSupplier.get(),
                        poseSupplier
                                .get()
                                .getRotation()
                                .plus(Rotation2d.fromRadians(-turretAngleSupplier.getAsDouble() + Math.PI)),
                        shooterTransform.getMeasureZ(),
                        MetersPerSecond.of(shooterVel),
                        shooterTransform.getRotation().getMeasureAngle()));
    }

    private int getOpenSpindexerSlot() {
        for (int i = 0; i < spindexerSlots.length; i++) {
            if (spindexerSlots[i] == null) {
                return i;
            }
        }
        return -1;
    }

    private int getSpindex(HeldSpeechBubble bubble) {
        for (int i = 0; i < spindexerSlots.length; i++) {
            if (spindexerSlots[i] == bubble) {
                return i;
            }
        }
        return -1;
    }

    private boolean isSpindexSlotNearTransfer(int slot, double spindexerPos) {
        if (slot < 0 || slot >= spindexerSlots.length) return false;

        double angle = MathUtil.inputModulus(spindexerPos, 0, 2 * Math.PI);
        double slotAngle = Units.degreesToRadians(slot * 72);

        return Math.abs(angle - slotAngle) < Units.degreesToRadians(15);
    }

    private Transform3d getSpindexerTransform(double spindexerYaw) {
        return new Transform3d(VisualizerConstants.M2_ZERO, new Rotation3d(0, 0, -spindexerYaw));
    }

    private Transform3d getBubbleInSpindexerTransform(double spindexerYaw, int slot) {
        return getSpindexerTransform(spindexerYaw)
                .plus(new Transform3d(
                        new Translation3d(SpindexerConstants.BUBBLE_TO_SPINDEXER, 0, 0.2)
                                .rotateBy(new Rotation3d(0, 0, -spindexerYaw)),
                        Rotation3d.kZero));
    }

    class HeldSpeechBubble {
        public Location location;
        public double x;
        public Transform3d transform;

        public HeldSpeechBubble() {
            location = Location.INTAKE;
            x = 0;
            transform = Transform3d.kZero;
        }

        public boolean update(double dt, double intakeVel, double spindexerPos, double transferVel, double shooterVel) {
            switch (location) {
                case INTAKE -> {
                    x += intakeVel * dt;
                    if (x > 1) {
                        int slot = getOpenSpindexerSlot();
                        if (slot < 0) break;
                        spindexerSlots[slot] = this;
                        location = Location.SPINDEXER;
                        break;
                    }
                    transform = new Transform3d(0.5 - x * 0.5, 0, 0.2 * x, Rotation3d.kZero);
                    break;
                }
                case SPINDEXER -> {
                    int index = getSpindex(this);
                    if (index < 0) {
                        location = Location.INTAKE;
                        break;
                    }
                    if (isSpindexSlotNearTransfer(index, spindexerPos) && transferVel > 1) {
                        spindexerSlots[index] = null;
                        location = Location.TRANSFER;
                        x = 2;
                        break;
                    }
                    transform = getBubbleInSpindexerTransform(spindexerPos, index);
                    break;
                }
                case TRANSFER -> {
                    x += transferVel * dt;
                    if (x > 3) {
                        location = Location.SHOOTER;
                        break;
                    }
                    transform = new Transform3d(-0.2, 0, 0.2 + (x - 2) * 0.2, Rotation3d.kZero);
                    break;
                }
                case SHOOTER -> {
                    x += shooterVel * dt;
                    if (x > 4) {
                        return true;
                    }
                    transform = new Transform3d(-0.2, (x - 3) * 0.2, 0.4, Rotation3d.kZero);
                }
            }
            return false;
        }
    }

    public enum Location {
        INTAKE, // 0-1
        SPINDEXER, // 1-2
        TRANSFER, // 2-3
        SHOOTER // 3-4
    }
}
