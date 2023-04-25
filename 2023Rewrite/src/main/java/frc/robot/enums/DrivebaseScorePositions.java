package frc.robot.enums;

import edu.wpi.first.math.geometry.Pose2d;

public enum DrivebaseScorePositions {
    
    NODE_0 ("Node 0", new Pose2d()),
    NODE_1 ("Node 1", new Pose2d()),
    NODE_2 ("Node 2", new Pose2d()),
    NODE_3 ("Node 3", new Pose2d()),
    NODE_4 ("Node 4", new Pose2d());


    private final String poseName;
    private final Pose2d desiredPose;

    DrivebaseScorePositions(String poseName, Pose2d desiredPose) {
        this.poseName    = poseName;
        this.desiredPose = desiredPose;
    }

    public String getName() {return poseName;}
    public Pose2d getPose() {return desiredPose;}
}
