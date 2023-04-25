// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.enums.ScorePositions;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.PathPlannerWrapper;
import frc.robot.subsystems.PneumaticInterface;
import frc.robot.subsystems.Vision;


public class RobotContainer {

  private final CommandXboxController driverController;
  private final CommandXboxController auxController;

  Drive              driveGlobalInstance;
  Manipulator        manipulatorGlobalInstance;
  PathPlannerWrapper pathPlannerWrapperGlobalInstance;
  PneumaticInterface pneumaticInterfaceGlobalInstance;
  AutoChooser        autoChooserGlobalInstance;
  Vision             visionGlobalInstance;


  public RobotContainer() {

    driveGlobalInstance              = Drive.getInstance();
    manipulatorGlobalInstance        = Manipulator.getInstance();
    pathPlannerWrapperGlobalInstance = PathPlannerWrapper.getInstance();
    pneumaticInterfaceGlobalInstance = PneumaticInterface.getInstance();
    autoChooserGlobalInstance        = AutoChooser.getInstance();
    visionGlobalInstance             = Vision.getInstance();

    driverController = new CommandXboxController(0);
    auxController    = new CommandXboxController(1);

    pathPlannerWrapperGlobalInstance.buildFullAuto();

    DriverStation.silenceJoystickConnectionWarning(true);

    CameraServer.startAutomaticCapture();
    
    setDefaultCommands();
    configureBindings();
  }

  private void setDefaultCommands() {
    driveGlobalInstance.setDefaultCommand(Commands.run(() -> Drive.getInstance().drive(
        new Translation2d(-MathUtil.applyDeadband(driverController.getLeftX(), 0.01),
                          -MathUtil.applyDeadband(driverController.getLeftY(), 0.01)),
                          -MathUtil.applyDeadband(driverController.getRightX(), 0.01)
    ), Drive.getInstance()));
  }

  private void configureBindings() {

    driverController.y().onTrue(Commands.run(
      () -> Drive.getInstance().resetGyro(), 
        Drive.getInstance()));

    driverController.rightBumper().whileTrue(Commands.run(
      () -> PneumaticInterface.getInstance().raiseIntakeArms(),
        PneumaticInterface.getInstance()));
    
    driverController.rightBumper().whileFalse(Commands.run(
      () -> PneumaticInterface.getInstance().lowerIntakeArms(),
        PneumaticInterface.getInstance()));
    
    driverController.x().onTrue(PathPlannerWrapper.getInstance().driveToPose(
      Drive.getInstance().getPose(), 
      TeleopPositionChooser.getInstance().getSelectedPosition().getPose(),
      Drive.getInstance().getLinearVelocity()
    ));

    


    auxController.rightStick().onTrue(Commands.runOnce(
      () -> PneumaticInterface.getInstance().openClaw(),
        PneumaticInterface.getInstance()));

    auxController.povDown().onTrue(Commands.runOnce(
      () -> PneumaticInterface.getInstance().closeClaw(),
        PneumaticInterface.getInstance()));

    //High
    auxController.rightBumper().onTrue(Commands.runOnce(
      () -> Manipulator.getInstance().setArmGoal(
          new TrapezoidProfile.State(
            ScorePositions.HIGH_GOAL.getTranslation().getAngle().getRadians(), 
            0
          )), Manipulator.getInstance()));
    
    auxController.b().onTrue(Commands.runOnce(
      () -> Manipulator.getInstance().setSlideGoal(
          new TrapezoidProfile.State(
            ScorePositions.HIGH_GOAL.getTranslation().getNorm(), 
            0
          )), Manipulator.getInstance()));

    //Mid
    auxController.back().onTrue(Commands.runOnce(
      () -> Manipulator.getInstance().setArmGoal(
          new TrapezoidProfile.State(
            ScorePositions.MID_GOAL.getTranslation().getAngle().getRadians(), 
            0
          )), Manipulator.getInstance()));

    auxController.x().onTrue(Commands.runOnce(
      () -> Manipulator.getInstance().setSlideGoal(
          new TrapezoidProfile.State(
            ScorePositions.MID_GOAL.getTranslation().getNorm(), 
            0
          )), Manipulator.getInstance()));

    //Low
    auxController.start().onTrue(Commands.runOnce(
      () -> Manipulator.getInstance().setArmGoal(
          new TrapezoidProfile.State(
            ScorePositions.LOW_GOAL.getTranslation().getAngle().getRadians(), 
            0
          )), Manipulator.getInstance()));

    auxController.y().onTrue(Commands.runOnce(
      () -> Manipulator.getInstance().setSlideGoal(
          new TrapezoidProfile.State(
            ScorePositions.LOW_GOAL.getTranslation().getNorm(), 
            0
          )), Manipulator.getInstance()));

    //Pickup
    auxController.a().onTrue(Commands.runOnce(
        () -> Manipulator.getInstance().setArmGoal(
            new TrapezoidProfile.State(
              ScorePositions.PICKUP_POS.getTranslation().getAngle().getRadians(), 
              0
            )), Manipulator.getInstance()).andThen(Commands.runOnce(
              () -> Manipulator.getInstance().setSlideGoal(
                  new TrapezoidProfile.State(
                    ScorePositions.PICKUP_POS.getTranslation().getNorm(), 
                    0
                  )), Manipulator.getInstance())));
    
    //Carry
    auxController.povUp().onTrue(Commands.runOnce(
      () -> Manipulator.getInstance().setArmGoal(
          new TrapezoidProfile.State(
            ScorePositions.CARRY_POS.getTranslation().getAngle().getRadians(), 
            0
          )), Manipulator.getInstance()).andThen(Commands.runOnce(
            () -> Manipulator.getInstance().setSlideGoal(
                new TrapezoidProfile.State(
                  ScorePositions.CARRY_POS.getTranslation().getNorm(), 
                  0
                )), Manipulator.getInstance())));
  }
}
