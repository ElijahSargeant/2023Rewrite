package frc.robot.auto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavX;

/**
 * Basic pathfinder made for our swerve
 */
public class Pathfinder {

    static Pathfinder instance;
    
    static Drive drive = Drive.getInstance();
    static NavX navX = NavX.getInstance();

    static double xFeetGoal;
    static double yFeetGoal;
    static double rotationGoal;
    static double hypDistance;
    static boolean run = false;
    static double totalDistance;
    static double xSpeed;
    static double ySpeed;
    static double zSpeed;
    static double speedMultiplier;
    static double distanceTraveled;

    NetworkTable dashboardTable;
    NetworkTableEntry distanceTraveledFt;


    public Pathfinder() {
       /* drive.getFrontLeft().resetSpeedDistance();
        drive.getFrontRight().resetSpeedDistance();
        drive.getBackLeft().resetSpeedDistance();
        drive.getBackRight().resetSpeedDistance();*/
        dashboardTable = NetworkTableInstance.getDefault().getTable("dashboard");

        distanceTraveledFt = dashboardTable.getEntry("distanceTraveled");
    }


    public void translate(double xFeet, double yFeet, double angle, double newSpeedMultiplier) {

        xFeetGoal = xFeet;
        yFeetGoal = yFeet;
        rotationGoal = angle;
        hypDistance = Math.hypot(xFeetGoal, yFeetGoal);
        speedMultiplier = newSpeedMultiplier;

        double distanceFrontLeft = drive.getFrontLeft().getDrivePosition();
        double distanceFrontRight = drive.getFrontRight().getDrivePosition();
        double distanceBackLeft = drive.getBackLeft().getDrivePosition();
        double distanceBackRight = drive.getBackRight().getDrivePosition();
        totalDistance = (distanceFrontLeft + distanceFrontRight + distanceBackLeft + distanceBackRight) / 4;
        totalDistance /= 7.643;
        
        run = true;

        System.out.println("Running path");
        while (run) {

            try {
                loop();
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        System.out.println("Stopping path");
    }

    
    public void loop() {


        if (run) {

            double distanceFrontLeft = drive.getFrontLeft().getDrivePosition();
            double distanceFrontRight = drive.getFrontRight().getDrivePosition();
            double distanceBackLeft = drive.getBackLeft().getDrivePosition();
            double distanceBackRight = drive.getBackRight().getDrivePosition();
            double averageDistance = (distanceFrontLeft + distanceFrontRight + distanceBackLeft + distanceBackRight) / 4;
            distanceTraveled = averageDistance / 7.643; 
            
            //double rotationOffset = (8.9 * (rotationGoal % 360) / 360) / 4;
            //System.out.println(distanceTraveled + " " + (totalDistance + hypDistance) + " " + hypDistance);
            //distanceTraveled -= rotationOffset;

            xSpeed = xFeetGoal / hypDistance;
            ySpeed = yFeetGoal / hypDistance;
            zSpeed = (rotationGoal - navX.getAngle()) * 0.005;

            distanceTraveledFt.setDouble(totalDistance);

            drive.move(xSpeed * speedMultiplier, ySpeed * speedMultiplier, zSpeed, navX.getAngle(), false);

            if(distanceTraveled >= (hypDistance + totalDistance)) {
                run = false;
            }
        }
    }

    public static Pathfinder getInstance() {

        if (instance == null) {

            instance = new Pathfinder();
        }

        return instance;
    }
}

    /*public void runPath() {
        double[][] points = pathWeezer.getTrajectory();

        int currentPoint = 1; 
        double pastX = points[0][3];
        double pastY = points[0][4];

        while(currentPoint < points.length) {
            if(!run) {
                currentPoint++;
                translate(points[currentPoint][3] - pastX, points[currentPoint][4] - pastY, points[currentPoint][5], 0.5);
            }
        }
    }

    public double getFtMoved() {
        return distanceTraveled;
    }
    
    public boolean getRun() {
        return run;
    }

    
}



/*package frc.robot.auto;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;


import java.io.IOException;
import java.nio.file.Path;

import frc.robot.Dashboard;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavX;

public class Pathfinder {

    static Pathfinder instance;

    enum pathState {

        running,
        end
    }

    pathState pState;

    Dashboard dash;

    Trajectory pathToRun;
    Drive drive;
    NavX navX;
    trajectoryController controller;
    
    int maxVelocity = 3; //m/s
    int maxAccel = 2;    //m/s^2
    int i = 1;

    SwerveModuleState[] states;

    double endTime;
    double elapsedTime = 0.001;
    double startTime;

    Trajectory traj = new Trajectory();

    public Pathfinder(String path) {

        drive = Drive.getInstance();

        navX = NavX.getInstance();

        controller = new trajectoryController();

        dash = Dashboard.getInstance();

        String trajJson = "paths/"+ path +".wpilib.json";

        Path trajPath = Filesystem.getDeployDirectory().toPath().resolve(trajJson);

        try {
            traj = TrajectoryUtil.fromPathweaverJson(trajPath);
        } catch (IOException e) {
            e.printStackTrace();
            DriverStation.reportError("Unable to open trajectory: " + trajJson, e.getStackTrace());
        }
    }

    public void drivePath() {

        /**
         * Steps for tuning 
         * 1. Have Robot drive 1M forward, compare to odometry and make adjustments to conversion factor as needed
         * 2. Figure out if NavX being a poopcock, or relative to azimuth degrees
         * 3. Have Robot turn 360 degrees, compare to odometry
         * 4. Run Diag path and tweak Holonomic P gain for X, Y, and Theta (assuming odom accurate enough)
         * 
         */

 /*       endTime = traj.getTotalTimeSeconds();

        startTime = System.currentTimeMillis();

        //List<State> stateList = traj.getStates();

        //for(pathToRun : stateList){
        while(elapsedTime <= endTime) {

            try {

                states = controller.updateVelocities(traj, elapsedTime);

                SwerveModuleState FL = states[0];
                SwerveModuleState FR = states[1];
                SwerveModuleState BL = states[2];
                SwerveModuleState BR = states[3];

                System.out.println("Speed: " + FL.speedMetersPerSecond + " Angle: " + FL.angle.getDegrees());

                
                drive.getFrontLeft().drivePathfinder(FL.speedMetersPerSecond, FL.angle.getDegrees());
                drive.getFrontRight().drivePathfinder(FR.speedMetersPerSecond, FR.angle.getDegrees());
                drive.getBackLeft().drivePathfinder(BL.speedMetersPerSecond, BL.angle.getDegrees());
                drive.getBackRight().drivePathfinder(BR.speedMetersPerSecond, BR.angle.getDegrees());
                
            } catch (Exception e) {
                
                pState = pathState.end;
                DriverStation.reportError("Can't Drive", e.getStackTrace());
            }

            elapsedTime = (System.currentTimeMillis() - startTime) / 1000;
        }

        drive.getFrontLeft().stop();
        drive.getFrontRight().stop();
        drive.getBackLeft().stop();
        drive.getBackRight().stop();

    }


}*/