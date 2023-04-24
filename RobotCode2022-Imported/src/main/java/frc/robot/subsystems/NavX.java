package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Robot;

public class NavX  implements Telemetry{

    static NavX instance;
    
    AHRS gyro;

    double offset;

    public NavX() {

        try {

            gyro = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {

			System.out.println("Error instantiating navX-MXP:  " + ex.getMessage());
        }
        
        gyro.reset();

        offset = 0;
    }

    /**
     * 
     * @return gyro angle w/ offset
     */
    public double getAngle() {

        return gyro.getAngle() % 360 + offset;
    }

    public void resetYaw() {

        gyro.zeroYaw();
    }


    public void setOffset(double offset) {

        this.offset = offset;
    }


    public static NavX getInstance() {
        
        if (instance == null) {
            
            instance = new NavX();
        }

        return instance;
    }

    @Override
    public void postData() {

        Robot.getTableInstance().getEntry("navXAngle").setDouble(getAngle());

        
    }

    @Override
    public void getData() {
        // TODO Auto-generated method stub
        
    }
}