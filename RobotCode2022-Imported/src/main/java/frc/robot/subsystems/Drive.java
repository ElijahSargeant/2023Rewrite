//Robot code from 2020

/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleConsumer;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.libs.swerve.SwerveDrive;
import frc.libs.swerve.WheelDrive;

import frc.robot.Constants;

/**
 * Drive class to manage chassis with swerve drive module.
 */

public class Drive implements Sendable{

    static Drive instance;

    static SwerveDrive chassis;

    static WheelDrive backRight;
    static WheelDrive backLeft;
    static WheelDrive frontRight;
    static WheelDrive frontLeft;

    DoubleConsumer BRP, BRI, BRD, BROFF;
    DoubleConsumer BLP, BLI, BLD, BLOFF;
    DoubleConsumer FRP, FRI, FRD, FROFF;
    DoubleConsumer FLP, FLI, FLD, FLOFF;


    public Drive() {

        backRight = new WheelDrive(Constants.DRIVE_BACK_RIGHT_AZIMUTH_ID, Constants.DRIVE_BACK_RIGHT_SPEED_ID, Constants.BR_AZIMUTH_ENCODER_ID, Constants.BR_ANGLE_OFFSET);

        backLeft = new WheelDrive(Constants.DRIVE_BACK_LEFT_AZIMUTH_ID, Constants.DRIVE_BACK_LEFT_SPEED_ID, Constants.BL_AZIMUTH_ENCODER_ID, Constants.BL_ANGLE_OFFSET);

        frontRight = new WheelDrive(Constants.DRIVE_FRONT_RIGHT_AZIMUTH_ID, Constants.DRIVE_FRONT_RIGHT_SPEED_ID, Constants.FR_AZIMUTH_ENCODER_ID, Constants.FR_ANGLE_OFFSET);

        frontLeft = new WheelDrive(Constants.DRIVE_FRONT_LEFT_AZIMUTH_ID, Constants.DRIVE_FRONT_LEFT_SPEED_ID, Constants.FL_AZIMUTH_ENCODER_ID, Constants.FL_ANGLE_OFFSET);

        chassis = new SwerveDrive(backRight, backLeft, frontRight, frontLeft);
    }

    @Override
    public void initSendable(SendableBuilder builder) {

        builder.setSmartDashboardType("RobotBase");

        builder.addDoubleProperty("BRSpeed", backRight.getSpeed(), null);
        builder.addDoubleProperty("BLSpeed", backLeft.getSpeed(), null);
        builder.addDoubleProperty("FRSpeed", frontRight.getSpeed(), null);
        builder.addDoubleProperty("FLSpeed", frontLeft.getSpeed(), null);

        builder.addDoubleProperty("BRAzimuth", backRight.getSendableAzimuthDegrees(), null);
        builder.addDoubleProperty("BLAzimuth", backLeft.getSendableAzimuthDegrees(), null);
        builder.addDoubleProperty("FRAzimuth", frontRight.getSendableAzimuthDegrees(), null);
        builder.addDoubleProperty("FLAzimuth", frontLeft.getSendableAzimuthDegrees(), null);

        builder.addDoubleProperty("BRP", backRight.getConstants()[0], PGain -> backRight.setP(PGain));
        builder.addDoubleProperty("BRI", backRight.getConstants()[1], IGain -> backRight.setI(IGain));
        builder.addDoubleProperty("BRD", backRight.getConstants()[2], DGain -> backRight.setD(DGain));
        builder.addDoubleProperty("BRAzimuthOffset", backRight.getConstants()[3], BROFF = offset -> backRight.setOffset(offset));

        builder.addDoubleProperty("BLP", backLeft.getConstants()[0], PGain -> backLeft.setP(PGain));
        builder.addDoubleProperty("BLI", backLeft.getConstants()[1], IGain -> backLeft.setI(IGain));
        builder.addDoubleProperty("BLD", backLeft.getConstants()[2], DGain -> backLeft.setD(DGain));
        builder.addDoubleProperty("BLAzimuthOffset", backLeft.getConstants()[3], BLOFF = offset -> backLeft.setOffset(offset));

        builder.addDoubleProperty("FRP", frontRight.getConstants()[0], FRP = PGain -> frontRight.setP(PGain));
        builder.addDoubleProperty("FRI", frontRight.getConstants()[1], FRI = IGain -> frontRight.setI(IGain));
        builder.addDoubleProperty("FRD", frontRight.getConstants()[2], FRD = DGain -> frontRight.setD(DGain));
        builder.addDoubleProperty("FRAzimuthOffset", frontRight.getConstants()[3], FROFF = offset -> frontRight.setOffset(offset));

        builder.addDoubleProperty("FLP", frontLeft.getConstants()[0], FLP = PGain -> frontLeft.setP(PGain));
        builder.addDoubleProperty("FLI", frontLeft.getConstants()[1], FLI = IGain -> frontLeft.setI(IGain));
        builder.addDoubleProperty("FLD", frontLeft.getConstants()[2], FLD = DGain -> frontLeft.setD(DGain));
        builder.addDoubleProperty("FLAzimuthOffset", frontLeft.getConstants()[3], FLOFF = offset -> frontLeft.setOffset(offset));
    }


    public void move(double forward, double strafe, double rotation, double angle, boolean isFieldOriented) {
        
        chassis.drive(forward, strafe, rotation * -1, angle, isFieldOriented);
    }

    public SwerveDrive getSwerve() {

        return chassis;
    }


    public WheelDrive getBackRight() {

        return backRight;
    }

    
    public WheelDrive getBackLeft() {

        return backLeft;
    }


    public WheelDrive getFrontRight() {

        return frontRight;
    }
    

    public WheelDrive getFrontLeft() {

        return frontLeft;
    }


    public static Drive getInstance() {

        if (instance == null) {
            
            instance = new Drive();
        }

        return instance;
    }
}