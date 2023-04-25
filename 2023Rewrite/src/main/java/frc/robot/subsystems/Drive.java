package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveModuleState2;
import swervelib.parser.SwerveParser;

public class Drive extends SubsystemBase {
    
    SwerveDrive swerveDriveInstance;

    private final Matrix<N3, N1> visionStdDevs;

    public SwerveDriveKinematics kinematics;

    static Drive driveInstance;

    public Drive() {
        try {
            swerveDriveInstance = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive();
            kinematics = swerveDriveInstance.kinematics;
        } catch (IOException e) {
            DriverStation.reportError("Swerve not Instantiated!", false);
            e.printStackTrace();
          }

        visionStdDevs = new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(
            0.01, 
            0.01, 
            Units.degreesToRadians(0.001));
    }

    public Pose2d getPose() {
        return swerveDriveInstance.getPose();
    }

    public void resetPose(Pose2d newPose) {
        swerveDriveInstance.resetOdometry(newPose);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        swerveDriveInstance.setModuleStates((SwerveModuleState2[]) states, false);
    }

    public void drive(Translation2d translation, double rotationRate) {

        swerveDriveInstance.drive(
            translation, 
            rotationRate, 
            true, 
            true, 
            true
        );
    }

    public void resetGyro() {
        swerveDriveInstance.zeroGyro();
    }

    public void addVisionMeasurement(Pose2d visionData, double timestamp) {
        swerveDriveInstance.addVisionMeasurement(visionData, timestamp, true, visionStdDevs);
    }

    public Command xLock() {
        return Commands.run(() -> swerveDriveInstance.lockPose());
    }

    public double getLinearVelocity() {
        return Math.hypot(swerveDriveInstance.getRobotVelocity().vxMetersPerSecond,
                          swerveDriveInstance.getRobotVelocity().vyMetersPerSecond);
    }

    public void periodic() {
        swerveDriveInstance.updateOdometry();
    }
    
    public static Drive getInstance() {

        if(driveInstance == null) {
            driveInstance = new Drive();
        } return driveInstance;
    }
}
