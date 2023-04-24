package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Dashboard {

    ShuffleboardTab pathTab, swerveTab;

    static Dashboard instance;

    public Dashboard() {

        pathTab = Shuffleboard.getTab("Pathfinder");
        swerveTab = Shuffleboard.getTab("Drivetrain");
    }


    public ShuffleboardTab getPathfinderTab() {

        return pathTab;
    }

    public ShuffleboardTab getDriveTab() {

        return swerveTab;
    }

    public static Dashboard getInstance() {

        if(instance == null) {

            instance = new Dashboard();
        }

        return instance;
    }
}