package frc.robot.enums;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.PneumaticInterface;

public enum AutoActions {
    
    OPEN_CLAW       ("OpenClaw",   Commands.run(() -> PneumaticInterface.getInstance().openClaw())),
    CLOSE_CLAW      ("CloseClaw",  Commands.run(() -> PneumaticInterface.getInstance().closeClaw())),
    INTAKE_DOWN     ("IntakeDown", Commands.run(() -> PneumaticInterface.getInstance().lowerIntakeArms())),
    INTAKE_UP       ("IntakeUp",   Commands.run(() -> PneumaticInterface.getInstance().raiseIntakeArms())),

    MANIP_TO_PICKUP ("ManipToPickup", Commands.run(() -> 
                        Manipulator.getInstance().setManipGoal(
                            ScorePositions.PICKUP_POS))),

    MANIP_TO_CARRY  ("ManipToPickup", Commands.run(() ->
                        Manipulator.getInstance().setManipGoal(
                            ScorePositions.CARRY_POS))),

    MANIP_TO_HIGH   ("ManipToPickup", Commands.run(() ->
                        Manipulator.getInstance().setManipGoal(
                            ScorePositions.HIGH_GOAL))),

    MANIP_TO_MID    ("ManipToPickup", Commands.run(() ->
                        Manipulator.getInstance().setManipGoal(
                            ScorePositions.MID_GOAL))),

    MANIP_TO_LOW    ("ManipToPickup", Commands.run(() ->
                        Manipulator.getInstance().setManipGoal(
                            ScorePositions.LOW_GOAL)));

    
    private final String  autoActionName;
    private final Command autoCommand;

    AutoActions(String autoActionName, Command autoCommand) {
        this.autoActionName = autoActionName;
        this.autoCommand    = autoCommand;
    }

    public String  actionName()    {return autoActionName;}
    public Command actionCommand() {return autoCommand;}
}
