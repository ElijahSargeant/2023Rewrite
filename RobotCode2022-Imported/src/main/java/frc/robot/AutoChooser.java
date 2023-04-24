package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.modes.DoNothing;
import frc.robot.auto.modes.DriveOffLine;
import frc.robot.auto.modes.Mode;
import frc.robot.auto.modes.ShootFiveBalls;
import frc.robot.auto.modes.ShootThreeBalls;
import frc.robot.auto.modes.ShootTwoBalls;

public class AutoChooser {

    static AutoChooser instance;

    SendableChooser<Modes> modeSelector;

    NetworkTable autoTable;
    NetworkTableEntry waitTime;
    NetworkTableEntry startOffset;

    enum Modes {
        DoNothing,
        ShootThreeBalls,
        ShootFiveBalls,
        ShootTwoBalls,
        DriveOffLine
    }

    public AutoChooser() {

        modeSelector = new SendableChooser<Modes>();

        autoTable = NetworkTableInstance.getDefault().getTable("auto");
        waitTime = autoTable.getEntry("waitTime");
        startOffset = autoTable.getEntry("startOffset");

        modeSelector.addOption("Do Nothing", Modes.DoNothing);
        modeSelector.addOption("Shoot Three Balls", Modes.ShootThreeBalls);
        modeSelector.addOption("Shoot Four Balls", Modes.ShootFiveBalls);
        modeSelector.addOption("Shoot Two Balls", Modes.ShootTwoBalls);
        modeSelector.addOption("Drive Off Line", Modes.DriveOffLine);

        SmartDashboard.putData(modeSelector);
    }

    public Mode getMode() {

        Modes result = modeSelector.getSelected();
        Mode modeToReturn = new DoNothing();

        switch (result) {
            case ShootThreeBalls:
                modeToReturn = new ShootThreeBalls();
                break;
            case ShootFiveBalls:
                modeToReturn = new ShootFiveBalls();
                break;
            case ShootTwoBalls:
                modeToReturn = new ShootTwoBalls();
                break;
            case DriveOffLine:
                modeToReturn = new DriveOffLine();
                break;
            default:
                modeToReturn = new DoNothing();
                break;
        }

        return modeToReturn;
    }

    public static AutoChooser getInstance() {

        if(instance == null) {

            instance = new AutoChooser();
        }

        return instance;
    }
}