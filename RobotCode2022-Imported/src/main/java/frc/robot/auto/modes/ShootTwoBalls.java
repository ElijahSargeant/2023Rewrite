package frc.robot.auto.modes;

import frc.robot.auto.ActionRunner;
import frc.robot.auto.Pathfinder;
import frc.robot.auto.actions.ShootBalls;
import frc.robot.subsystems.Sucker;

public class ShootTwoBalls extends Mode{

    //Pathfinder path = new Pathfinder("TwoBallPath");

    ActionRunner actionRunner = ActionRunner.getInstance();
    Sucker sucker = Sucker.getInstance();
    Pathfinder pathfinder = Pathfinder.getInstance();

    public void run() {

        System.out.println("Running two ball");
        sucker.deploy();
        sucker.suckOrBlow(0.75);
        pathfinder.translate(7, -5, -85, 0.2);
        sucker.retract();
        pathfinder.translate(-1, 3, -85, 0.21);
        sucker.suckOrBlow(0);
        actionRunner.run(new ShootBalls(2, 2000, -2000));

    }
}