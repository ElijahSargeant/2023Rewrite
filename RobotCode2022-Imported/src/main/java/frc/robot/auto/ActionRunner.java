/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import frc.robot.auto.actions.Action;

/**
 * Add your docs here.
 */
public class ActionRunner {

    static ActionRunner instance;

    static Action currentAction;
    static boolean run = true;


    public void run(Action action) {

        currentAction = action;

        System.out.println("Starting " + action.getClass().getName() + " action!");

        if (run) {

            action.start();

            while (currentAction != null) {

                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            action.stop();
        }

        System.out.println("Stopped " + action.getClass().getName() + " action!");
    }


    public void loop() {

        if (currentAction != null) {
            
            if (currentAction.loop() == false) { //Run action loop and get results

                currentAction = null;
            };
        }
    }
    

    public void start() { //So we can run in testing

        run = true;
    }


    public void forceStop() { //possibly changes this so the loop above reads from a state variable instead of firing this funcion
    
        if (currentAction != null) {

            currentAction.stop();
        }

        currentAction = null;
        run = false;
    }
    

    public static ActionRunner getInstance() {

        if (instance == null) {

            instance = new ActionRunner();
        }

        return instance;
    }
}