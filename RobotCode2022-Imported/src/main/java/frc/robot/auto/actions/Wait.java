package frc.robot.auto.actions;


public class Wait implements Action {

    long time;

    public Wait(long time) {

        this.time = time;
    }

    @Override
    public void start() {
        
    }

    @Override
    public boolean loop() {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return false;
    }

    @Override
    public void stop() {
        
    }
}