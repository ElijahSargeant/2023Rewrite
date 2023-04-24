package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;





public class Climber{
    
    DigitalInput leftLimitSwitch;
    DigitalInput rightLimitSwitch;
    DigitalInput climberLockLimitSwitch;

    NetworkTable dashboardTable;
    NetworkTableEntry leftLimit;
    NetworkTableEntry rightLimit;

    DoubleSolenoid solenoidErect = new DoubleSolenoid(Constants.COMPRESSOR_ID, PneumaticsModuleType.REVPH, 14, 15);
    DoubleSolenoid solenoid1 = new DoubleSolenoid(Constants.COMPRESSOR_ID, PneumaticsModuleType.REVPH, 12, 13);
    DoubleSolenoid solenoid2 = new DoubleSolenoid(Constants.COMPRESSOR_ID, PneumaticsModuleType.REVPH, 11, 10);

    static Climber instance;

    Sucker suck = Sucker.getInstance();

    Spark winch;

    public Climber() {

        winch = new Spark(Constants.WINCH_MOTOR_ID);

        leftLimitSwitch = new DigitalInput(1);
        rightLimitSwitch = new DigitalInput(3);
        climberLockLimitSwitch = new DigitalInput(2);

        dashboardTable = NetworkTableInstance.getDefault().getTable("dashboard");

        leftLimit = dashboardTable.getEntry("leftLimit");
        rightLimit = dashboardTable.getEntry("rightLimit");
    }

    public void startPos() {

        solenoidErect.set(Value.kReverse);
        solenoid1.set(Value.kReverse);
        solenoid2.set(Value.kForward);
    }

    public void stop() {

        winch.set(0);
    }


public void deploy(){
 
    solenoidErect.set(Value.kForward);

    if(leftLimitSwitch.get())
        leftLimit.setString("Left Limit got");
    else
        leftLimit.setString("Left Limit not got");

    if(climberLockLimitSwitch.get()) 
        rightLimit.setString("Climb Lock Got");
    else
        rightLimit.setString("Climb Lock not got");

    suck.deploy();
    

   if(leftLimitSwitch.get()) {

        winch.set(-1);
        solenoid2.set(Value.kReverse);
    }


    if(!rightLimitSwitch.get() && leftLimitSwitch.get()) {
        winch.set(-1);
    }

    if(rightLimitSwitch.get()) {

        winch.set(0);
        solenoid1.set(Value.kForward);

        if(!climberLockLimitSwitch.get()) {
    
            solenoid2.set(Value.kForward);
        }

        //solenoid1.set(Value.kReverse);
    }

}

    public static Climber getInstance(){

        if (instance == null) {

            instance = new Climber();
        }
        
        return instance;
        
    }

}