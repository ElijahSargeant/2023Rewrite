package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.RobotLEDState;

public class LEDState extends SubsystemBase {

    static LEDState ledStateInstance;

    private final Spark blinkinLED; 

    private RobotLEDState currentState, nextState;
    
    public LEDState() {

        currentState = RobotLEDState.IDLE;
        nextState    = RobotLEDState.IDLE;

        blinkinLED = new Spark(1);
    }

    public void setRobotState(RobotLEDState state) {
        nextState = state;
    }


    public void periodic() {

        //if priority overrides
        if( (nextState.getPriority() < currentState.getPriority()) && isStateActive(currentState) ) {
            //transition
            currentState = nextState;
        } else if (!isStateActive(currentState)) {
            currentState = nextState;
        }

        //update
        blinkinLED.set(currentState.getColor());
        
        SmartDashboard.putString("Current LED State", currentState.toString());
    }

    public boolean isStateActive(RobotLEDState currState) {
        for(RobotLEDState state : RobotLEDState.values()) {
            if (currState == state)
                return state.isRunning();
        } return false;
    }


    public static LEDState getInstance() {
        if(ledStateInstance == null) {
            ledStateInstance = new LEDState();
        } return ledStateInstance;
    }
}
