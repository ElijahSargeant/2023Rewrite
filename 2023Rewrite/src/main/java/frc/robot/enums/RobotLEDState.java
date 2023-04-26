package frc.robot.enums;

public enum RobotLEDState {
    
    IDLE         (0, 99, false),
    WANTS_CUBE   (0, 50, false),
    WANTS_CONE   (0, 40, false),
    AUTO_DRIVING (0, 10, false),
    SCORING      (0, 20, false),
    INTAKING     (0, 30, false);

    int priority;
    int ledColor;

    boolean isRunning;

    /**
     * 
     * @param ledColor The desired color
     * @param priority The lower the priority the more important, i.e. 3 will override 5
     * @param isRunning if the desired state is currently being displayed
     */
    RobotLEDState(int ledColor, int priority, boolean isRunning) {
        this.ledColor = ledColor;
        this.priority = priority;

        this.isRunning = isRunning;
    }

    public int getColor() {
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
