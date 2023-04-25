package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.enums.DrivebaseScorePositions;

public class TeleopPositionChooser {
    
    private final SendableChooser<DrivebaseScorePositions> positionChooser;

    static TeleopPositionChooser teleopPositionChooserInstance;

    public TeleopPositionChooser() {
        positionChooser = new SendableChooser<>();

        boolean first = true;

        for(DrivebaseScorePositions positions : DrivebaseScorePositions.values()) {
            if(first) {
                positionChooser.setDefaultOption(positions.name(), positions);
                first = false;
            }  else {
                positionChooser.addOption(positions.name(), positions);
            }
        }
    }

    public DrivebaseScorePositions getSelectedPosition() {
        return positionChooser.getSelected();
    }

    public static TeleopPositionChooser getInstance() {
        if(teleopPositionChooserInstance == null) {
            teleopPositionChooserInstance = new TeleopPositionChooser();
        } return teleopPositionChooserInstance;
    }
}
