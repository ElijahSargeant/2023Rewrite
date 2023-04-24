package frc.robot.auto;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavX;

public class NewFinder {

    static NewFinder instance;

    double[][] coords;

    Odometry odom;
    NavX gyro;
    Drive drive;

    boolean run;
    int i;

    double currentxPos = 0;
    double currentyPos = 0;
    double currentAngle = 0;

    double hypotDistance = 1;
    
    
    public NewFinder() {

        odom = new Odometry();
        gyro = NavX.getInstance();
        drive = Drive.getInstance();

        run = false;
    }

    /**
     * Coords set up as following:
     * [*][0] holds x pos ft
     * [*][1] holds y pos ft
     * [*][2] holds angle pos degrees
     * 
     * @param coordinates desired path and heading of robot
     */
    public void traversePath(double[][] coordinates) {

        coords = coordinates;

        i = 0;

        run = true;

        System.out.println("Running path");

        while(run) {

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

        if(run) {

            if(i < coords.length) {

                if(hypotDistance >= 0) {

                    currentxPos  = odom.getxyCoord()[0];
                    currentyPos  = odom.getxyCoord()[1];
                    currentAngle = gyro.getAngle();

                    //in ft and degrees
                    double xError = coords[i][0] - currentxPos;
                    double yError = coords[i][1] - currentyPos;
                    double angleError = coords[i][2] - currentAngle;

                    double hypotDistance = Math.hypot(xError, yError);

                    drive.move(xError/hypotDistance, yError/hypotDistance, angleError / 360 , currentAngle, true);

                } else { i += 1;}
            } else { run = false;}            
        }
    }

    public static NewFinder getInstance() {

        if (instance == null) {

            instance = new NewFinder();
        }

        return instance;
    }


}
