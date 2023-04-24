package frc.libs.swerve;

public class SwerveDrive {

	boolean deadband;

	public double L = 25;//length of wheel axle distances
	public double W = 25;//width of wheel axle distances

	double r;

	public final double CONTROLLER_DEADBAND = .05;

	public double[] getDimensions() {

		double dimensions[] = {L, W};

		return dimensions;
	}
	
	/**
	 * Use for human input
	 * @param x1 Forward input
	 * @param y1 Strafe input
	 * @param rotation Azimuth input
	 * @param theta Gyro Yaw input
	 * @param robotCentric 
	 */
	public void drive (double x1, double y1, double rotation, double theta, boolean robotCentric) {


		if (CONTROLLER_DEADBAND * -1 < x1 && x1 < CONTROLLER_DEADBAND && CONTROLLER_DEADBAND * -1 < y1 && 
			y1 < CONTROLLER_DEADBAND && CONTROLLER_DEADBAND * -1 < rotation && rotation < CONTROLLER_DEADBAND){

			deadband = true;
		}

		else deadband = false;

		double temp;
		double strafe;
		double forward;

		r = Math.hypot(L, W);
	
		if (robotCentric) {

			forward = x1;
			strafe  = y1;
		}

		else {

			temp = y1 * Math.cos(Math.toRadians(theta)) + x1 * Math.sin(Math.toRadians(theta)); //allows for field centric control
		
			strafe = -y1 * Math.sin(Math.toRadians(theta)) + x1 * Math.cos(Math.toRadians(theta));

			forward = temp;
		}
		
		double a = strafe - rotation  * (L / r); //placeholder vector values
		
		double b = strafe + rotation  * (L / r);
		
		double c = forward - rotation * (W / r);
		
		double d = forward + rotation * (W / r);

		
		double backRightSpeed = 0; //calculating speed
				
		double backLeftSpeed = 0;
				
		double frontRightSpeed = 0;
				
		double frontLeftSpeed = 0;
		
		
		//Output is 0 to 1
		backRightSpeed = Math.hypot(a, c);
			
		backLeftSpeed = Math.hypot(a, d);

		frontRightSpeed = Math.hypot(b, c);
					
		frontLeftSpeed = Math.hypot(b, d);
		

		//Output is 0 to 5 volts
		double backRightAngle 	= (((Math.atan2(a, c) / Math.PI) * 2.5) + 2.5) + backRight.getOffset(); 
		
		double backLeftAngle 	= (((Math.atan2(a, d) / Math.PI) * 2.5) + 2.5) + backLeft.getOffset();
		//									-1 to 1		   -2.5 to 2.5   0 to 5  
		double frontRightAngle	= (((Math.atan2(b, c) / Math.PI) * 2.5) + 2.5) + frontRight.getOffset();

		double frontLeftAngle	= (((Math.atan2(b, d) / Math.PI) * 2.5) + 2.5) + frontLeft.getOffset();

		//normalize wheel speeds
        double max = backRightSpeed;

        if (backLeftSpeed > max)    max = backLeftSpeed;
        if (frontLeftSpeed > max)   max = frontLeftSpeed;
        if (frontRightSpeed > max)  max = frontRightSpeed;

        if (max > 1) {

            backRightSpeed  /= max;
            backLeftSpeed   /= max;
            frontLeftSpeed  /= max;
            frontRightSpeed /= max;
		}
		
			backRight.drive(backRightSpeed, backRightAngle, deadband); //just using a class to organize modules together
		
			backLeft.drive(backLeftSpeed, backLeftAngle, deadband);
		
			frontRight.drive(frontRightSpeed, frontRightAngle, deadband);
		
			frontLeft.drive(frontLeftSpeed, frontLeftAngle, deadband);
	}

	/**Set the robot to a specified angle (in degrees)
	 * @param gyroAngle The navX's  measured angle
	 * @param setpointAngle The desired setpoint angle
	 */
	public void rotateToAngle(double gyroAngle, double setpointAngle) {

		r = Math.hypot(L, W);

		double gyroError = (gyroAngle % 360) - setpointAngle;

		double kP = 0.3;

				//Output is 0 to 1
		double gyroSpeedSetpoint = gyroError * kP;

		//3.125 = the rotaion angle required
		double gyroAngleSetpoint = 3.125;

		backRight.drive(gyroSpeedSetpoint, gyroAngleSetpoint + backRight.getOffset(), false);
		backLeft.drive(gyroSpeedSetpoint, gyroAngleSetpoint + backLeft.getOffset(), false);
		frontRight.drive(gyroSpeedSetpoint, gyroAngleSetpoint + frontRight.getOffset(), false);
		frontLeft.drive(gyroSpeedSetpoint, gyroAngleSetpoint + frontLeft.getOffset(), false);
	}


		private WheelDrive backRight;
		
		private WheelDrive backLeft;
		
		private WheelDrive frontRight;
		
		private WheelDrive frontLeft;
		
		
		public SwerveDrive (WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft ) {
			
			//passing into the full full swerve class
			this.backRight = backRight;
			
			this.backLeft = backLeft;
			
			this.frontRight = frontRight;
			
			this.frontLeft = frontLeft;
		}
}