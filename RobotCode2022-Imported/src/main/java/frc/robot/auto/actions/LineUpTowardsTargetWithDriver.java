/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import frc.robot.Constants;
import frc.robot.DriverController;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Vision;

/**
 * Rotate shooter side of robot towards target with driver forward and strafe input
 */
public class LineUpTowardsTargetWithDriver implements Action {
    
    DriverController driverController = DriverController.getInstance();
    Drive drive = Drive.getInstance();
    Vision vision = Vision.getInstance();
    NavX navX = NavX.getInstance();

    //double lastError = 0;

    double targetAngle;
    double offset;
    int dir;


    public void start() {

        vision.lightOn();
        targetAngle = navX.getAngle() - vision.getHorizontalOffset();
    }


    public boolean loop() {
        offset = vision.getHorizontalOffset();

        //System.out.println("FL: " + drive.getFrontLeft().getAzimuth() + " FR: " + drive.getFrontRight().getAzimuth() + " BL: " + drive.getBackLeft().getAzimuth() + " BR: " + drive.getBackRight().getAzimuth());

        //if(offset < 2.5 && offset > -2.5) 
        //    offset = 0;

        //drive.move(offset * Constants.AUTO_ROTATE_P, 0, 0.625, navX.getAngle(), false);

        drive.getFrontLeft().drive(offset * Constants.AUTO_ROTATE_P, 0.631, false);
        drive.getFrontRight().drive(offset * Constants.AUTO_ROTATE_P, 2.805, false);
        drive.getBackLeft().drive(offset * Constants.AUTO_ROTATE_P, 2.775, false);
        drive.getBackRight().drive(offset * Constants.AUTO_ROTATE_P, 3.967, false);

        return true; 
    }

    
    public void stop() {
        
        vision.lightOn();
        //lastError = 0;
    }
}