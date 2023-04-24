package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    
    NetworkTable limelightTable;
    NetworkTableEntry robotPose;

    private double[] robotPoseDouble;
    private double limelightPipelineTimestamp;

    private Pose2d estimatedRobotPose;
    
    static Vision visionInstance;

    public Vision() {

        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void periodic() {

        robotPoseDouble = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[]{});

        if (robotPoseDouble.length > 0) {

            estimatedRobotPose = new Pose2d(robotPoseDouble[0], robotPoseDouble[1], Rotation2d.fromDegrees(robotPoseDouble[5]));

            limelightPipelineTimestamp = Timer.getFPGATimestamp() - (robotPoseDouble[6]/1000.0);

            SmartDashboard.putNumber("Pose X",  estimatedRobotPose.getX());
            SmartDashboard.putNumber("Pose Y",  estimatedRobotPose.getY());
            SmartDashboard.putNumber("Heading", estimatedRobotPose.getRotation().getDegrees());
            SmartDashboard.putNumber("Timestamp", limelightPipelineTimestamp);

            Drive.getInstance().addVisionMeasurement(estimatedRobotPose, limelightPipelineTimestamp);
        }
    }

    public static Vision getInstance() {
        if(visionInstance == null) {
            visionInstance = new Vision();
        } return visionInstance;
    }
}
