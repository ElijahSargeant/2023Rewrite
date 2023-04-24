package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticInterface extends SubsystemBase {
    
    static PneumaticInterface pneumaticInterfaceInstance;

    Compressor compressor;
    PneumaticHub pneumaticHub;

    private final DoubleSolenoid clawSolenoid, intakeArms;

    public PneumaticInterface() {

        compressor = new Compressor(40, PneumaticsModuleType.REVPH);
        pneumaticHub = new PneumaticHub(40);

        clawSolenoid = pneumaticHub.makeDoubleSolenoid(9, 8);
        intakeArms = pneumaticHub.makeDoubleSolenoid(11, 12);

        compressor.enableDigital();
    }

    public void openClaw() {
        clawSolenoid.set(Value.kForward);
    }

    public void closeClaw() {
        clawSolenoid.set(Value.kReverse);
    }

    public void lowerIntakeArms() {
        intakeArms.set(Value.kReverse);
    }

    public void raiseIntakeArms() {
        intakeArms.set(Value.kForward);
    }


    public static PneumaticInterface getInstance() {
        if(pneumaticInterfaceInstance == null) {
            pneumaticInterfaceInstance = new PneumaticInterface();
        } 
        return pneumaticInterfaceInstance;
    }
}
