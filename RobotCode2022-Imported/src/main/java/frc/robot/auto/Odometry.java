package frc.robot.auto;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavX;

public class Odometry {

    Drive swerve;
    NavX gyro;

    double lastTime;
    double xPos;
    double yPos;

    /**
     * Uses instantiated drive subsystem to get all modules
     */
    public Odometry() {

        swerve = Drive.getInstance();

        gyro = NavX.getInstance();

        lastTime = 0;
        xPos = 0;
        yPos = 0;
    }

    public double[] getxyCoord() {

        //placeholder values to average state
        double a = ((Math.sin(swerve.getBackRight().getAzimuthRadians()) * swerve.getBackRight().getSpeedPositionFeet()) + (Math.sin(swerve.getBackLeft().getAzimuthRadians()) * swerve.getBackLeft().getSpeedPositionFeet())) / 2;
        double b = ((Math.sin(swerve.getFrontLeft().getAzimuthRadians()) * swerve.getFrontLeft().getSpeedPositionFeet()) + (Math.sin(swerve.getFrontRight().getAzimuthRadians()) * swerve.getFrontRight().getSpeedPositionFeet())) / 2;
        double c = ((Math.sin(swerve.getFrontRight().getAzimuthRadians()) * swerve.getFrontRight().getSpeedPositionFeet()) + (Math.sin(swerve.getBackRight().getAzimuthRadians()) * swerve.getBackRight().getSpeedPositionFeet())) / 2;
        double d = ((Math.sin(swerve.getFrontLeft().getAzimuthRadians()) * swerve.getFrontLeft().getSpeedPositionFeet()) + (Math.sin(swerve.getBackLeft().getAzimuthRadians()) * swerve.getBackLeft().getSpeedPositionFeet())) / 2;
        
        //radians / sec
        double rotation1 = (b-a)/swerve.getSwerve().getDimensions()[0];
        double rotation2 = (c-d)/swerve.getSwerve().getDimensions()[1];
        double rotation  = (rotation1 + rotation2) / 2;

        double forward1 =  rotation * (swerve.getSwerve().getDimensions()[0] / 2) + a;
        double forward2 = -rotation * (swerve.getSwerve().getDimensions()[0] / 2) + b;
        double forward  =  (forward1 + forward2) / 2;

        double strafe1 = rotation * (swerve.getSwerve().getDimensions()[1] / 2) + c;
        double strafe2 = rotation * (swerve.getSwerve().getDimensions()[1] / 2) + d;
        double strafe  = (strafe1 + strafe2) / 2;

        double forwardFinal = forward * Math.cos(Math.toRadians(gyro.getAngle())) + Math.sin(Math.toRadians(gyro.getAngle()));
        double strafeFinal = strafe * Math.cos(Math.toRadians(gyro.getAngle())) - forward * Math.sin(Math.toRadians(gyro.getAngle()));

        //timeStep 
        
        double timeStep = System.currentTimeMillis() - lastTime;

        xPos = xPos + forwardFinal * timeStep;
        yPos = yPos + strafeFinal * timeStep;

        double[] xyCoord = {xPos, yPos};

        lastTime = System.currentTimeMillis();
        return xyCoord;
    }
}
