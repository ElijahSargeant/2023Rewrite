package frc.robot.enums;

public enum RobotLEDState {
    
    //aqua
    IDLE         (0.87, 70, false),
    //green
    DS_ATTACHED  (0.77, 90, false),
    FMS_ATTACHED (0.77, 80, false),
    //violet
    WANTS_CUBE   (0.91, 50, false),
    //yellow
    WANTS_CONE   (0.69, 40, false),
    //red
    AUTO_DRIVING (0.61, 10, false),
    //orange
    SCORING      (0.65, 20, false),
    //white
    INTAKING     (0.93, 30, false);


    int priority;

    double ledColor;

    boolean isRunning;

    /**
     * 
     * @param ledColor The desired color
     * @param priority The lower the priority the more important, i.e. 3 will override 5
     * @param isRunning if the desired state is currently being displayed
     */
    RobotLEDState(double ledColor, int priority, boolean isRunning) {
        this.ledColor = ledColor;
        this.priority = priority;
        this.isRunning = isRunning;
    }

    public double getColor() {
        return ledColor;
    }

    public int getPriority() {
        return priority;
    }

    public boolean setIsRunning(boolean isRunning) {
        this.isRunning = isRunning;

        return true;
    }

    public boolean isRunning() {
        return isRunning;
    }
}
