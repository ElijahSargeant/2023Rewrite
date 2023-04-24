package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveModuleState2;
import swervelib.parser.SwerveParser;

public class Drive extends SubsystemBase {

    static Drive driveInstance;
    SwerveDrive swerveDriveInstance;

    public SwerveDriveKinematics kinematics;

    public Drive() {
        try {
            swerveDriveInstance = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive();
            kinematics = swerveDriveInstance.kinematics;
        } catch (IOException e) {
            DriverStation.reportError("Swerve not Instantiated!", false);
            e.printStackTrace();
          }
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

    public Command xLock() {
        return Commands.run(() -> swerveDriveInstance.lockPose());
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
