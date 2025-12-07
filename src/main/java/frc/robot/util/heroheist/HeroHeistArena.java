package frc.robot.util.heroheist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import org.ironmaple.simulation.SimulatedArena;

public class HeroHeistArena extends SimulatedArena {
    public static final class HeroFieldObstaclesMap extends FieldMap {
        public HeroFieldObstaclesMap() {
            super();

            super.addBorderLine(new Translation2d(0, 0), new Translation2d(FieldConstants.FIELD_LENGTH, 0));
            super.addBorderLine(new Translation2d(0, 0), new Translation2d(0, FieldConstants.FIELD_WIDTH));
            super.addBorderLine(
                    new Translation2d(FieldConstants.FIELD_LENGTH, 0),
                    new Translation2d(FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH));
            super.addBorderLine(
                    new Translation2d(0, FieldConstants.FIELD_WIDTH),
                    new Translation2d(FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH));
        }
    }

    public HeroHeistArena() {
        super(new HeroFieldObstaclesMap());
    }

    @Override
    public void placeGamePiecesOnField() {
        for (Pose2d notePosition : FieldConstants.BLUE_BUBBLE_STARTING_POSITIONS)
            super.addGamePiece(new BlueBubbleOnField(notePosition.getTranslation()));
        for (Pose2d notePosition : FieldConstants.RED_BUBBLE_STARTING_POSITIONS)
            super.addGamePiece(new RedBubbleOnField(notePosition.getTranslation()));
    }
}
