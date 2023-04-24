package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.AutoActions;
import frc.robot.subsystems.Drive;

public class PathPlannerWrapper {

    public static PathPlannerWrapper pathPlannerInstance;

    HashMap<String, Command> eventMap;

    SwerveAutoBuilder autoBuilder;
    
    public PathPlannerWrapper() {
        PathPlannerServer.startServer(5811);

        eventMap = new HashMap<>();

        for(AutoActions action : AutoActions.values()) {
            eventMap.put(action.actionName(), action.actionCommand());
        }
    }

    public void buildFullAuto() {

        autoBuilder = new SwerveAutoBuilder(
            Drive.getInstance()::getPose, 
            Drive.getInstance()::resetPose, 
            Drive.getInstance().kinematics,
            // PID constants to correct for translation error (used to create the X and Y PID controllers) 
            new PIDConstants(5.0, 0.0, 0.0), 
            // PID constants to correct for rotation error (used to create the rotation controller)
            new PIDConstants(0.5, 0.0, 0.0), 
            Drive.getInstance()::setModuleStates, 
            eventMap,
            true, 
            Drive.getInstance()
        ); 
    }

    public SwerveAutoBuilder getAutoBuilder() {
        return autoBuilder;
    }

    public HashMap<String, Command> getEventMap() {
        return eventMap;
    }

    public static PathPlannerWrapper getInstance() {
        if(pathPlannerInstance == null) {
            pathPlannerInstance = new PathPlannerWrapper();
        } return pathPlannerInstance;
    }
}
