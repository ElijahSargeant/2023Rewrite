package frc.robot.enums;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public enum ScorePositions {
    
    PICKUP_POS (new Translation2d(3.0, Rotation2d.fromRadians(0))),
                //Negative Pi to flip theta back around origin
    CARRY_POS  (new Translation2d(-4.0, Rotation2d.fromRadians(-Math.PI))),
    LOW_GOAL   (new Translation2d(7, Rotation2d.fromRadians(0.67))),
    MID_GOAL   (new Translation2d(18, Rotation2d.fromRadians(1.59))),
    HIGH_GOAL  (new Translation2d(36.7, Rotation2d.fromRadians(1.82)));

    private final Translation2d desiredPose;

    ScorePositions(Translation2d desiredPose) {
        this.desiredPose = desiredPose;
    }

    public Translation2d getTranslation() {return desiredPose;}
}
