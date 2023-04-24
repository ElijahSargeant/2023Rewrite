package frc.robot.auto.modes;

import frc.robot.auto.ActionRunner;
import frc.robot.auto.NewFinder;
import frc.robot.auto.Pathfinder;
import frc.robot.auto.actions.ShootBalls;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Sucker;

public class ShootThreeBalls extends Mode {

    ActionRunner actionrunner = ActionRunner.getInstance();
    Pathfinder pathfinder = Pathfinder.getInstance();
    Sucker sucker = Sucker.getInstance();
    NavX navX = NavX.getInstance();
    NewFinder path = NewFinder.getInstance();

    double[][] firstCoords = {{2, 3, 45}, {7,-2, 180}};

    public void run() {

        System.out.println("Running three ball");
        sucker.deploy();
        sucker.suckOrBlow(1);
        path.traversePath(firstCoords);
        //pathfinder.translate(0, 4.3, 180, 0.22);
        sucker.retract();
        sucker.suckOrBlow(0.1);
        //pathfinder.translate(0, -2, 190, 0.2);
        sucker.deploy();
        sucker.suckOrBlow(1);
        actionrunner.run(new ShootBalls(2, 2020, -2020));
        //pathfinder.translate(7.5, -.1, 270, 0.5);
        sucker.retract();
        sucker.suckOrBlow(0.1);
        //pathfinder.translate(-6, 1, 200, 0.5);
        sucker.suckOrBlow(0);
        sucker.deploy();
        sucker.suckOrBlow(0.5);
        actionrunner.run(new ShootBalls(1, 2000, -2000));
    }
}