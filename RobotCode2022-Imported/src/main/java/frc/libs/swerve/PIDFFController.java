package frc.libs.swerve;

import frc.robot.Robot;

public class PIDFFController {
    

    double kP;
    double kI;
    double kD;

    double PSet;
    double ISet;
    double DSet;

    double kS;
    double kV;
    double kA;

    double error;
    double prevError;
    double returnValue;

    double previousSetpoint; 
    double optoAngle;

    boolean continuous;
    boolean invertSpeed;

    int min;
    int max;

    double clUnitsAway;
    double ccUnitsAway;

    double returnVal;

    double[] setPos = {0,0};
    double[] setpoints = new double[2];


    /**
     * **NEO ONLY** PID and feedforward controller
     * @param kP constant
     * @param kI constant
     * @param kD constant
     * @param kS constant
     * @param kV constant
     * @param kA constant
     */
    public PIDFFController(double kP, double kI, double kD) {

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        prevError = 0;

        min = 0;
        max = 0;

        previousSetpoint = 0;
        optoAngle = 0;

        setpoints[0] = 0;
        setpoints[1] = 1;

        continuous = false;
        invertSpeed = false;

        returnVal = 0;
    }

    public void isContinuous(int min, int max) {

        continuous = true;

        this.min = min;
        this.max = max;
    }

    public double calculateFF(double error) {

        //Feedforward explanation code
		//double kS;
		//double kV;
		//double kA;

		//kS****************************************************************************************************
        kS = 0.78; //static friction of motor. measured in volts

		//kV****************************************************************************************************
		//double freeSpeedVelocity    = 5676; //from rev website, in RPM
		double wheelDiameter        = .25; //in feet
		double azimuthGearReduction = 32; //from Colin

		//double maxVelocity = (freeSpeedVelocity * Math.PI * wheelDiameter) / azimuthGearReduction;
        double maxVoltage= 12;
        double scalingFactor = 4.8;

		kV = scalingFactor * error; 


        //kA****************************************************************************************************
        
        double stallTorque = 2.6; //from rev website, in Nm
        double robotMass   = 124; //in lbs, prolly less now

		double maxAcceleration = (2 * stallTorque * azimuthGearReduction) / (wheelDiameter * robotMass);
		
        kA = (maxVoltage) / (maxAcceleration);
        
        return kS;
    }

    /**
     * Lil helper function to get optimized angle wheel speed
     * @return returns whether or not speed should be inverted
     */
    public boolean speedInverted() {

        return invertSpeed;
    }

    /**
     * Calculates smallest distance for wheel to rotate towards
     * @param feedback error in angle (units is volts)
     * @param setpoint desired angle (units in volts)
     * @return returns gain factor scaled against volts
     */
    public double calculate(double feedback, double setpoint) {

        if (continuous) { 
            
            //ex feedback says we're at 3.5v, setpoint is 1.5v
            if (feedback > setpoint) {

                //if we're more than 180 degrees away (in this case we are, at 2v away)
               if (Math.abs(feedback - setpoint) > 1.25) {

                    //if the setpoint is greater than 180 degrees we take off 180, otherwise add on 180
                    //so following above values our new setpoint is 4v
                    setpoint = (setpoint >= 2.5) ? (setpoint -= 2.5) : (setpoint += 2.5);

                    //make sure we invert the speed
                    invertSpeed = true;
                }  else invertSpeed = false;
                
                

                //cl ex (5v - 3.5v) + 4v = 5.5v
                clUnitsAway = (max - feedback) + setpoint; //Going right
                //cc ex (3.5v - 0v) - 4v = -0.5v
                ccUnitsAway = (feedback - min) - setpoint; //Going left

                
            }

            //ex feeback at 1.5v, setpoint at 3.5v
            else if (feedback < setpoint) {

                //if we're more than 180 degrees away
                if (Math.abs(setpoint - feedback) > 1.25) {

                    //if the setpoint is greater than 180 degrees we take off 180, otherwise add on 180
                    //so following above values our new setpoint is 1v
                    setpoint = (setpoint >= 2.5) ? (setpoint -= 2.5) : (setpoint += 2.5);

                    //make sure we invert the speed
                    invertSpeed = true;
                }  else invertSpeed = false;

                //cl ex (1v - 1.5v) = -0.5v
                clUnitsAway = (setpoint - feedback);
                //cc ex (1.5v - 0v) + (5v - 3.5v) = 3v
                ccUnitsAway = (feedback - min) + (max - setpoint);
            }

            //taking minimum distance as our best fit
            error = Math.min(clUnitsAway, ccUnitsAway);

            //gotta make sure it spins the right way
            if (error == ccUnitsAway) {

                error *= -1;
            }
        }else {

            error = setpoint - feedback;
        }
        
        //create simple PID loop to get er where she needs to go
        PSet = error * kP;
        ISet += error;
        DSet = (error - prevError) / Robot.kDefaultPeriod;

        prevError = error;

        returnVal = PSet + (ISet * kI) + (DSet * kD);

        //normalize so we don't freak out the motor controllers with a magnitude above 1
        if (returnVal > 1) {

            returnVal = 1;
        }

        if (returnVal < -1) {

            returnVal = -1;
        }

        return returnVal;
    }
}
