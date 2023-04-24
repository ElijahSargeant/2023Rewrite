package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class Pneumatics{

    static Pneumatics instance;

    Compressor compressor;


    public Pneumatics() {

        compressor = new Compressor(Constants.COMPRESSOR_ID, PneumaticsModuleType.REVPH);
    }


    //Returns true if the pressure is low
    public boolean getPressureSwitch() {

        return compressor.getPressureSwitchValue();
    }


    public void runCompressor() {

        compressor.enableDigital();
    }


    public void stopCompressor() {

        compressor.disable();
    }

    
    static public Pneumatics getInstance() {

        if (instance == null) {

            instance = new Pneumatics();
        }

        return instance;
    }
}