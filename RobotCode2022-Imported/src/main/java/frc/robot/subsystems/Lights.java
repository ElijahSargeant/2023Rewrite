package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Lights{

    static Lights instance;

    Spark ledStrip;

    String allianceColor;

    public Lights() {
        ledStrip = new Spark(9);

    }

    public void setLEDs(double colorValue) {
        if (colorValue == 0 || colorValue < -99 || colorValue > 99) {
            ledStrip.set(0.93);
        }
        else if (colorValue >= -0.99 && colorValue <= 0.99) {
            ledStrip.set(colorValue);
        }
        else if (colorValue >= -99 && colorValue <= 99) {
            ledStrip.set(0.01 * colorValue);
        }
    }

    public void setLEDs(String allianceColor){
        if (allianceColor.equalsIgnoreCase("red")) {
            ledStrip.set(0.61);
        }
        else if (allianceColor.equalsIgnoreCase("blue")) {
            ledStrip.set(0.87);
        }
        else {
            ledStrip.set(0.69); //Yellow
        }
    }

    public static Lights getInstance(){

        if (instance == null) {

            instance = new Lights();
        }

        return instance;
    }

}