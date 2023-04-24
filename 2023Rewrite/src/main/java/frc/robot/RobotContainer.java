// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.PathPlannerWrapper;
import frc.robot.enums.ScorePositions;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.PneumaticInterface;


public class RobotContainer {

  private final CommandXboxController driverController;
  private final CommandXboxController auxController;

  Drive              driveGlobalInstance;
  PathPlannerWrapper pathPlannerWrapperGlobalInstance;
  PneumaticInterface pneumaticInterfaceGlobalInstance;
  AutoChooser        autoChooserGlobalInstance;


  public RobotContainer() {

    driveGlobalInstance              = Drive.getInstance();
    pathPlannerWrapperGlobalInstance = PathPlannerWrapper.getInstance();
    pneumaticInterfaceGlobalInstance = PneumaticInterface.getInstance();
    autoChooserGlobalInstance        = AutoChooser.getInstance();

    driverController = new CommandXboxController(0);
    auxController    = new CommandXboxController(1);

    pathPlannerWrapperGlobalInstance.buildFullAuto();
    
    setDefaultCommands();
    configureBindings();
  }

  private void setDefaultCommands() {
    driveGlobalInstance.setDefaultCommand(Commands.run(() -> Drive.getInstance().drive(
        new Translation2d(-MathUtil.applyDeadband(driverController.getLeftX(), 0.01),
                          -MathUtil.applyDeadband(driverController.getLeftY(), 0.01)),
                          -MathUtil.applyDeadband(driverController.getRightX(), 0.01)
    )));
  }

  private void configureBindings() {

    driverController.y().onTrue(Commands.run(
      () -> Drive.getInstance().resetGyro()));

    driverController.rightBumper().whileTrue(Commands.run(
      () -> PneumaticInterface.getInstance().raiseIntakeArms()));
    
    driverController.rightBumper().whileFalse(Commands.run(
      () -> PneumaticInterface.getInstance().lowerIntakeArms()));

    


    auxController.rightStick().onTrue(Commands.runOnce(
      () -> PneumaticInterface.getInstance().openClaw()));

    auxController.povDown().onTrue(Commands.runOnce(
      () -> PneumaticInterface.getInstance().closeClaw()));

    //High
    auxController.rightBumper().onTrue(Commands.runOnce(
      () -> Manipulator.getInstance().setArmGoal(
          new TrapezoidProfile.State(
            ScorePositions.HIGH_GOAL.getTranslation().getAngle().getRadians(), 
            0
          ))));
    
    auxController.b().onTrue(Commands.runOnce(
      () -> Manipulator.getInstance().setSlideGoal(
          new TrapezoidProfile.State(
            ScorePositions.HIGH_GOAL.getTranslation().getNorm(), 
            0
          ))));

    //Mid
    auxController.back().onTrue(Commands.runOnce(
      () -> Manipulator.getInstance().setArmGoal(
          new TrapezoidProfile.State(
            ScorePositions.MID_GOAL.getTranslation().getAngle().getRadians(), 
            0
          ))));

    auxController.x().onTrue(Commands.runOnce(
      () -> Manipulator.getInstance().setSlideGoal(
          new TrapezoidProfile.State(
            ScorePositions.MID_GOAL.getTranslation().getNorm(), 
            0
          ))));

    //Low
    auxController.start().onTrue(Commands.runOnce(
      () -> Manipulator.getInstance().setArmGoal(
          new TrapezoidProfile.State(
            ScorePositions.LOW_GOAL.getTranslation().getAngle().getRadians(), 
            0
          ))));

    auxController.y().onTrue(Commands.runOnce(
      () -> Manipulator.getInstance().setSlideGoal(
          new TrapezoidProfile.State(
            ScorePositions.LOW_GOAL.getTranslation().getNorm(), 
            0
          ))));

    //Pickup
    auxController.a().onTrue(Commands.runOnce(
        () -> Manipulator.getInstance().setArmGoal(
            new TrapezoidProfile.State(
              ScorePositions.PICKUP_POS.getTranslation().getAngle().getRadians(), 
              0
            ))).andThen(Commands.runOnce(
              () -> Manipulator.getInstance().setSlideGoal(
                  new TrapezoidProfile.State(
                    ScorePositions.PICKUP_POS.getTranslation().getNorm(), 
                    0
                  )))));
    
    //Carry
    auxController.povUp().onTrue(Commands.runOnce(
      () -> Manipulator.getInstance().setArmGoal(
          new TrapezoidProfile.State(
            ScorePositions.CARRY_POS.getTranslation().getAngle().getRadians(), 
            0
          ))).andThen(Commands.runOnce(
            () -> Manipulator.getInstance().setSlideGoal(
                new TrapezoidProfile.State(
                  ScorePositions.CARRY_POS.getTranslation().getNorm(), 
                  0
                )))));
  }
}
