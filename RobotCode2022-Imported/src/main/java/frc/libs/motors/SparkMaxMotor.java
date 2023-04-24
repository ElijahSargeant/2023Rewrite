/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.libs.motors;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Add your docs here.
 */
public class SparkMaxMotor {

    private CANSparkMax controller;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    
    public SparkMaxMotor(int motorId) {

        controller = new CANSparkMax(motorId, MotorType.kBrushless);

        controller.restoreFactoryDefaults();
        controller.setSmartCurrentLimit(40);
        
        pidController = controller.getPIDController();
        encoder = controller.getEncoder();
    }


    public void setSpeed(double speed) {
    
        controller.set(speed);
    }


    public void setRPMVelocity(int RPM) {

        pidController.setReference(RPM, ControlType.kVelocity);
        
    }


    public double getVelocity() {

        return Math.abs(encoder.getVelocity());
    }


    public void configurePID(double p, double i, double d, double ff, double IZone) {

        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(ff);
        pidController.setIZone(IZone);
    }

    
    public void burnFlash() {

        controller.burnFlash();
    }


    public void SetRPMVelocity(int flop) {
    }
}