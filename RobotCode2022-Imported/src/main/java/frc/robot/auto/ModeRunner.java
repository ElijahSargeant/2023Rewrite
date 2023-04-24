package frc.robot.auto;

import frc.robot.auto.modes.Mode;

public class ModeRunner extends Thread {

    Mode mode;

    public ModeRunner(Mode newMode) {

        mode = newMode;
    }

    public void run() {

        try {
            Thread.sleep(mode.waitTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        mode.run();
    }
}