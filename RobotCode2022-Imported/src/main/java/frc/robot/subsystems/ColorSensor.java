package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;

public class ColorSensor {
    
    static ColorSensor instance;

    ColorSensorV3 colorSensor;

    public ColorSensor() {

        colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    }

    /**
     * Largest value when object closest.
     * Range: 0-2047
     */
    public double getDistance() {

        return colorSensor.getProximity();
    }

    public double getIR() {

        return colorSensor.getIR();
    }


    public static ColorSensor getInstance() {

        if (instance == null)
            return instance = new ColorSensor();
        
        else return instance;
    }
}
