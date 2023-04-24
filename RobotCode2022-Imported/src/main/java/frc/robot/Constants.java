/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//For now constants not relating to the shooter or drive/swerve isn't needed.
//Constants might be subject to change.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Constants {
   
    //Drive subsystem constants
    public static final double ETPF = 10; //Encoder Ticks Per Foot
    
	public static final int WHEEL_BASE_WIDTH = 25;
    public static final int WHEEL_BASE_DEPTH = 25;
    
    public static final int DRIVE_FRONT_LEFT_SPEED_ID = 2;//7;
    public static final int DRIVE_FRONT_LEFT_AZIMUTH_ID = 3;//8;
    public static final double FL_P = 0.75;
    public static final double FL_I = 0;
    public static final double FL_D = 0.003;
    public static final int FL_AZIMUTH_ENCODER_ID = 3;
	public static double FL_ANGLE_OFFSET = 1.24;//1.54;

    public static final int DRIVE_FRONT_RIGHT_SPEED_ID = 8;//6;
    public static final int DRIVE_FRONT_RIGHT_AZIMUTH_ID = 9;//5;
    public static final double FR_P = 0.75;
    public static final double FR_I = 0;
    public static final double FR_D = 0.003;
    public static final int FR_AZIMUTH_ENCODER_ID = 2;
	public static double FR_ANGLE_OFFSET =  2.13;//2.89; //41;

    public static final int DRIVE_BACK_LEFT_SPEED_ID = 10;//1;
    public static final int DRIVE_BACK_LEFT_AZIMUTH_ID = 11;//2;
    public static final double BL_P = 0.85;
    public static final double BL_I = 0;
    public static final double BL_D = 0.003;
    public static final int BL_AZIMUTH_ENCODER_ID = 0;  
	public static double BL_ANGLE_OFFSET = 0.88;//0.37;

    public static final int DRIVE_BACK_RIGHT_SPEED_ID = 6;//4;
    public static final int DRIVE_BACK_RIGHT_AZIMUTH_ID = 7;//3;
    public static final double BR_P = 0.75;
    public static final double BR_I = 0;
    public static final double BR_D = 0.003;
    public static final int BR_AZIMUTH_ENCODER_ID = 1;
    public static double BR_ANGLE_OFFSET = 2.10;//4.25;3.276

    public static final double PATHFINDER_KP = 0;
	public static final double PATHFINDER_KV = 0.0909090909;
	public static final double PATHFINDER_KA = 0.011;

    public static final double DRIVE_CORRECTION_FT = 7.643;

    //Shooter subsystem constants
    public static final double SHOOTER_HOOD_P = 0.01;
    public static final int SHOOTER_GATE_ID = 0;
    public static final int SHOOTER_ID_1 = 16;
    public static final int SHOOTER_ID_2 = 12;
    public static final int HOOD_ID = 3;
    public static final double SHOOTER_KP = 0.00007;
    public static final double SHOOTER_KD = 0.00002;
    public static final double SHOOTER_KF = 0.00021; //0.00019
    public static final int INDEX_PROX = 9;
    //Shooter threshold was 100 RPM

    //Intake subsystem constants
    public static final int INTAKE_ID = 20;
    public static final int DEPLOY_ID_1 = 8;
    public static final int DEPLOY_ID_2 = 9;

    //Vision subsystem constants
    public static final double VISION_TARGET_HEIGHT = 104.5;
    public static final double VISION_CAMERA_HEIGHT = 37.5;//1.7493;
    public static final double VISION_CAMERA_ANGLE = 20;//25.3778;

    //Pneumatics constants
    public static final int COMPRESSOR_ID = 40;

    //Auto Constants
    public static final double AUTO_FORWARD_P = 0.2;
    public static final double AUTO_ROTATE_P = 0.022;
    public static final double AUTO_ROTATE_D = 0.009; //change this back to 0.009 once you put the WheelDrive slow
    public static final double AUTO_STRAFE_P = 0.05;
    public static final double AUTO_GYRO_P = -0.03;

    //Pathfinder Constants
    public static final double PATHFINDER_SLOWDOWN = 0;
    public static final double PATHFINDER_SLOWDOWN_END = 0.1;
    public static final double PATHFINDER_SLOWDOWN_P = 0.5;

    public static final PneumaticsModuleType LOCK_SOLENOID_ID_1 = null;

    public static final int LOCK_SOLENOID_ID_2 = 0;

    public static final int WINCH_MOTOR_ID = 5;

    //Color Sensor Constants
    public static final double MIN_PROX_VALUE = 0;
    public static final double BLUE_MIN_COLOR = 0;
    public static final double BLUE_MAC_COLOR = 0;
    public static final double RED_MIN_COLOR = 0;
    public static final double RED_MAX_COLOR = 0;

    //62 Constants total, 49 constants in use
}