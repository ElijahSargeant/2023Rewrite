package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.Autos;
import frc.robot.subsystems.PathPlannerWrapper;

public class AutoChooser {
    
    private final SendableChooser<Autos> autoChooser;

    static AutoChooser autoChooserInstance;

    public AutoChooser() {
        autoChooser = new SendableChooser<>();

        boolean first = true;

        for(Autos modes : Autos.values()) {
            if(first) {
                autoChooser.setDefaultOption(modes.name(), modes);
                first = false;
            }  else {
                autoChooser.addOption(modes.name(), modes);
            }
        }
    }

    public Command getSelectedMode() {

        return 
            PathPlannerWrapper.getInstance().getAutoBuilder().fullAuto(
                PathPlanner.loadPathGroup(
                    autoChooser.getSelected().path(), 
                    autoChooser.getSelected().constraints()
                )
            );
    }

    public static AutoChooser getInstance() {
        if(autoChooserInstance == null) {
            autoChooserInstance = new AutoChooser();
        } return autoChooserInstance;
    }
}
