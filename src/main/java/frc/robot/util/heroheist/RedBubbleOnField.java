package frc.robot.util.heroheist;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.dyn4j.geometry.Circle;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class RedBubbleOnField extends GamePieceOnFieldSimulation {
    public static final GamePieceInfo RED_BUBBLE_INFO = new GamePieceInfo(
            "Red Speech Bubble",
            new Circle(Units.inchesToMeters(3.5)), // .176 for algae
            Inches.of(7),
            Ounces.of(5),
            1.8,
            5,
            0.8 * 0.25);

    public RedBubbleOnField(Translation2d initialPosition) {
        super(RED_BUBBLE_INFO, new Pose2d(initialPosition, new Rotation2d()));
    }
}
