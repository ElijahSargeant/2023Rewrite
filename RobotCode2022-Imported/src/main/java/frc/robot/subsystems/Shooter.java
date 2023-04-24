package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;

import java.util.Queue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 

public class Shooter {

    static final int rpmChangeConst = 20;

    static final double maxRPM = 5676;

    static Shooter instance;

    CANSparkMax flywheelBlack; 
    CANSparkMax flywheelBlue;
    Spark hoodMotor;
    Spark greenWheel;

    SparkMaxPIDController blackPID;
    SparkMaxPIDController bluePID;

    RelativeEncoder blackEnc;
    RelativeEncoder blueEnc;

    NetworkTable dashboardTable;
    NetworkTableEntry blackRPM;
    NetworkTableEntry blueRPM;

    NetworkTableEntry blackSetpoint;
    NetworkTableEntry blueSetpoint;

    double blackRPMSetpoint;
    double blueRPMSetpoint;

    int rpmChange = 0;

    Vision vision;
    Drive drive = Drive.getInstance();
    ColorSensor colorSensor = ColorSensor.getInstance();


    String allianceColor;
    Queue<Integer> ballQueue;

    
    Color bColor;
    DigitalInput ballInTopPos;
    int ballsin = 0;

    boolean first = true;
    double startTime;

    int boostIncr = 0;

    public Shooter() {
        vision = Vision.getInstance();

        flywheelBlack = new CANSparkMax(Constants.SHOOTER_ID_1, MotorType.kBrushless);
        flywheelBlue = new CANSparkMax(Constants.SHOOTER_ID_2, MotorType.kBrushless);
        hoodMotor = new Spark(Constants.HOOD_ID);
        greenWheel = new Spark(Constants.SHOOTER_GATE_ID);

        ballInTopPos = new DigitalInput(Constants.INDEX_PROX);
        

        blackPID = flywheelBlack.getPIDController();
        bluePID = flywheelBlue.getPIDController();

        bluePID.setFF(0.00021);
        bluePID.setP(0.00006);
        bluePID.setD(0.00071);

        blackPID.setFF(0.0002);
        blackPID.setP(0.00025);
        blackPID.setD(0.0006);

        blackEnc = flywheelBlack.getEncoder();
        blueEnc = flywheelBlue.getEncoder();

        dashboardTable = NetworkTableInstance.getDefault().getTable("dashboard");

        blackRPM = dashboardTable.getEntry("blackRPM");
        blueRPM = dashboardTable.getEntry("blueRPM");

        blackSetpoint = dashboardTable.getEntry("blackSetpoint");
        blueSetpoint = dashboardTable.getEntry("blueSetpoint");

        blackSetpoint.setDouble(0);
        blueSetpoint.setDouble(0);

    }

    public void setcolor(String aColor) {

        allianceColor = aColor;
        System.out.println(allianceColor);
    }

    public void loop() {

        //if we have a ball in the gate
        /*if(colorSensor.getProximity() < Constants.MIN_PROX_VALUE) {
            //add it to the queue
            ballQueue.add(colorSensor.getIR());

            ballsin++;
        }

        //while the gate is empty
        if(!ballInTopPos.get() && ballsin < 1) {

            greenWheel.set(0.5);
        }*/

        if(getProx()) {

            greenWheel.set(0);
        } else greenWheel.set(.5);
    }

    public void pewpew() {

        /*double x = vision.getDistanceFromTarget();

        if(x < 12) {

            blueRPMSetpoint =  -17.078 * Math.pow(x,2) + 282.73 * x - 3007.7;

            blackRPMSetpoint = 37.333*(x) + 1550.6;
        } else {

            blueRPMSetpoint = -41.346 * Math.pow(x,2) + 890.32 * x -7645.9 - boostIncr;

            blackRPMSetpoint = 9.0978*Math.pow(x,3) - 445.94*Math.pow(x,2) + 7078.1*x -35070;
        }*/

        blueRPMSetpoint = -1 * (26.186 * Math.pow(vision.getDistanceFromTarget(), 2) - 449.39 * vision.getDistanceFromTarget() + 3816.4 + boostIncr);
        //blackRPMSetpoint = 80.016 * vision.getDistanceFromTarget() + 1171.4;

        blackRPMSetpoint = 0.3214 * Math.pow(vision.getDistanceFromTarget(), 5) - 18.482 * Math.pow(vision.getDistanceFromTarget(), 4)   
        + 415.11 * Math.pow(vision.getDistanceFromTarget(), 3) - 4537.3 * Math.pow(vision.getDistanceFromTarget(), 2) + 24138 * vision.getDistanceFromTarget() -48162 - boostIncr;

        blackPID.setReference(blackRPMSetpoint, ControlType.kVelocity);
        bluePID.setReference(blueRPMSetpoint, ControlType.kVelocity);

        

        if(blackEnc.getVelocity() >= blackRPMSetpoint + 10 && blueEnc.getVelocity() <= blueRPMSetpoint - 10) {

            greenWheel.set(1);
        } else greenWheel.set(0);

        

        blackRPM.setDouble(blackEnc.getVelocity());
        blueRPM.setDouble(blueEnc.getVelocity());

        blackSetpoint.setDouble(blackRPMSetpoint);
        blueSetpoint.setDouble(blueRPMSetpoint);
    }

    public void pewpewAuto(double blackAutoRPM, double blueAutoRPM, boolean isLinedUp) {

        if(isLinedUp) {

            blackPID.setReference(blackAutoRPM, ControlType.kVelocity);
            bluePID.setReference(blueAutoRPM, ControlType.kVelocity);
        
        

        if(blackEnc.getVelocity() >= blackRPMSetpoint + 10 && blueEnc.getVelocity() <= blueRPMSetpoint - 10) {

            greenWheel.set(1);
        } else {
            greenWheel.set(0);
        }

        } else {

            greenWheel.set(0);
        }

        

        blackRPM.setDouble(blackEnc.getVelocity());
        blueRPM.setDouble(blueEnc.getVelocity());

        blackSetpoint.setDouble(blackRPMSetpoint);
        blueSetpoint.setDouble(blueRPMSetpoint);
    }

    public void safeShot() {

        blackPID.setReference(800, ControlType.kVelocity);
        bluePID.setReference(-5500, ControlType.kVelocity);

        if(blackEnc.getVelocity() >= 800 && blueEnc.getVelocity() <= -5500) {

            greenWheel.set(1);
        } else {
            greenWheel.set(0);
        }

        blackSetpoint.setDouble(blackEnc.getVelocity());
        blueSetpoint.setDouble(blueEnc.getVelocity());
    }


    public void idle() {
            blackPID.setReference(1400, ControlType.kVelocity);
            bluePID.setReference(-1800, ControlType.kVelocity);
    }

    public void stop() {
        greenWheel.set(0);
    }

    public Boolean getProx() {

        return !ballInTopPos.get();
    }

    public void badBall() {

        blackPID.setReference(1500, ControlType.kVelocity);
        bluePID.setReference(-1500, ControlType.kVelocity);

        greenWheel.set(1);
    }

    public void raiseBoost() {

        boostIncr += 10;
    }

    public void lowerBoost() {

        boostIncr -= 10;
    }

    public double getBlackRPM() {

        return blackEnc.getVelocity();
    }

    public double getBlueRPM() {

        return blueEnc.getVelocity();
    }

    public double getBlackRPMSetpoint() {

        return blackRPMSetpoint;
    }

    public int getRPMChange() {

        return rpmChange;
    }

    public void setRPMChange(int newRPMVal) {

        rpmChange = newRPMVal;
    }

    public void addToRPM() {

        rpmChange += 20;
    }

    public void subtractFromRPM() {

        rpmChange -= 20;
    }

    public void test(boolean shoot) {

       blackPID.setReference(blackSetpoint.getDouble(blackRPMSetpoint), ControlType.kVelocity);
       bluePID.setReference(blueSetpoint.getDouble(blueRPMSetpoint), ControlType.kVelocity);

       if(blackEnc.getVelocity() >= blackRPMSetpoint && blueEnc.getVelocity() <= blueRPMSetpoint && shoot) {

            greenWheel.set(1);
        } else greenWheel.set(0);
    }

    public static Shooter getInstance() {

        if (instance == null) {

            instance = new Shooter();
        }

        return instance;
    }

}