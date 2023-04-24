package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Sucker{

    static Sucker instance;

    DoubleSolenoid intakeSolenoid;
    CANSparkMax intake;



    public Sucker() {

        intakeSolenoid = new DoubleSolenoid(Constants.COMPRESSOR_ID, PneumaticsModuleType.REVPH, Constants.DEPLOY_ID_1, Constants.DEPLOY_ID_2);

        intake         = new CANSparkMax(Constants.INTAKE_ID, MotorType.kBrushless);
    }


    public void deploy() {

        intakeSolenoid.set(Value.kForward);
    }


    public void retract() {

        intakeSolenoid.set(Value.kReverse);
    }


    public void suckOrBlow(double speed) {

        intake.set(speed);
    }

    public static Sucker getInstance(){

        if (instance == null) {

            instance = new Sucker();
        }

        return instance;
    }

}