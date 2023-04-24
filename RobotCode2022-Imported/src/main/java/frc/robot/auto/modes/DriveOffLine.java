package frc.robot.auto.modes;

import frc.robot.auto.ActionRunner;
import frc.robot.auto.Pathfinder;
import frc.robot.auto.actions.ShootBalls;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Sucker;

public class DriveOffLine extends Mode {

    ActionRunner actionrunner = ActionRunner.getInstance();
    Pathfinder pathfinder = Pathfinder.getInstance();
    Sucker sucker = Sucker.getInstance();
    NavX navX = NavX.getInstance();
    
    public void run() {

        System.out.println("Running two ball");
        sucker.deploy();
        sucker.suckOrBlow(1);
        pathfinder.translate(0, 4.3, 180, 0.22);
        sucker.retract();
        sucker.suckOrBlow(0.1);
        pathfinder.translate(0, -2, 190, 0.2);
        sucker.deploy();
        sucker.suckOrBlow(1);
        actionrunner.run(new ShootBalls(2, 2020, -2020));
    }
}
