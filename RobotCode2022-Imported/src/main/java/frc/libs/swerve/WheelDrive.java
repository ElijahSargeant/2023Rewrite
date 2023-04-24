package frc.libs.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;
 
public class WheelDrive {
	
	
private CANSparkMax angleMotor;
private CANSparkMax speedMotor;

public RelativeEncoder driveEncoder; //public CANEncoder driveEncoder;

private PIDFFController anglePID;
private SparkMaxPIDController velocityPID;

private AnalogInput azimuthEncoder;

double P;
double I;
double D;

double ANGLE_OFFSET;

double setpoint;

double velocitySetpoint;

//convert m/s to rpm AT MOTOR and vice versa
double velocityRPMConversion = 1517.6538819;

	/**
	 * @param angleMotor The CAN ID of the azimuth controller
	 * @param speedMotor The CAN ID of the speed controller
	 * @param analogIn   The Analog ID of the azimuth encoder
	 * @param P          Proportional error for the PID loop
	 * @param I          Integrated error for the PID loop
	 * @param D          Derivative error for the PID loop
	 * @param ANGLE_OFFSET The module's encoder offset
	 */
	public WheelDrive(int angleMotor, int speedMotor, int analogIn, double offset) {

		// create our "wheels"
		this.angleMotor = new CANSparkMax(angleMotor, MotorType.kBrushless);
		this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);


		//anglePID = new PIDFFController(getConstants()[0].getAsDouble(), getConstants()[1].getAsDouble(), getConstants()[2].getAsDouble());
		anglePID = new PIDFFController(0.7412, 0, 0);
		anglePID.isContinuous(0, 5);

		this.azimuthEncoder = new AnalogInput(analogIn);

		this.driveEncoder = this.speedMotor.getEncoder();

		ANGLE_OFFSET = offset;//getConstants()[3].getAsDouble();

		velocityPID = this.speedMotor.getPIDController();
		velocityPID.setP(5e-4);
		velocityPID.setI(0);
		velocityPID.setD(0);
		velocityPID.setFF(0.000156);
		velocityPID.setOutputRange(-1, 1);

		//this.angleMotor.setSmartCurrentLimit(20);
	}

	public void drivePathfinder(double velocity, double angle) {

		angle /= 72;
		angle += getOffset();

		if (angle > 5) angle -= 5;
		if (angle < 0) angle += 5;

		//maybe correct magic number? at 3.74 m/s RPM is 5676 adjusted for gear ratio
		velocitySetpoint = Math.abs(velocityRPMConversion * velocity);

		velocityPID.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);

		setpoint = anglePID.calculate(azimuthEncoder.getVoltage(), angle);

		angleMotor.set(setpoint);

		//System.out.println("ANGLE SETPOINT: " + setpoint);

		//System.out.println("Motor " + angleMotor.getDeviceId() + " Angle: " + getAzimuth());
	}


	public void drive(double speed, double angle, boolean deadband) {
		
		//normalizes the encoder angle in case offsets caused it to go above 5
		if (angle > 5) angle -= 5;
		if (angle < 0) angle += 5;

		//System.out.println("ID: " + speedMotor.getDeviceId() + " Value: " + azimuthEncoder.getVoltage());

		if (deadband) {

			speedMotor.set(0);
			angleMotor.set(0);
		}

		else { 

			setpoint = anglePID.calculate(azimuthEncoder.getVoltage(), angle);

			//checks to see if speed needs to be reversed for optimized angle
			speedMotor.set(anglePID.speedInverted() ? speed *= -1 : speed);
			angleMotor.set(setpoint);
		}

	}
	
	public double getAzimuth() {

		return azimuthEncoder.getVoltage();
	}

	public double getAzimuthDegrees() {

		return (azimuthEncoder.getVoltage() + ANGLE_OFFSET) * 72;
	}

	public double getAzimuthRadians() {

		return (azimuthEncoder.getVoltage() + ANGLE_OFFSET) * ((2 * Math.PI) / 5);
	}

	public double getSpeedPositionFeet() {
		return driveEncoder.getPosition() / Constants.DRIVE_CORRECTION_FT;
	}

	public DoubleSupplier getSpeed() {

		DoubleSupplier speed = () -> {

			return driveEncoder.getVelocity();
		};

		return speed;
	}

	public DoubleSupplier getSendableAzimuthDegrees() {

		DoubleSupplier angle = () -> {

			return azimuthEncoder.getVoltage() * 72;
		};

		return angle;
	}

	public DoubleSupplier[] getConstants() {

		DoubleSupplier pGain = () -> {

			return P;
		};

		DoubleSupplier iGain = () -> {

			return I;
		};

		DoubleSupplier dGain = () -> {

			return D;
		};

		DoubleSupplier offset = () -> {

			return ANGLE_OFFSET;
		};

		DoubleSupplier[] constants = {pGain, iGain, dGain, offset};

		return constants;
	}

	public void setP(double gain) {

		P = gain;
	}

	public void setI(double gain) {

		I = gain;
	}

	public void setD(double gain) {

		D = gain;
	}

	public void setOffset(double offset) {

		ANGLE_OFFSET = offset;
	}

	public void resetSpeedDistance() {
		driveEncoder.setPosition(0);
	}

	public void setAzimuth(double setpoint) {

		angleMotor.set(anglePID.calculate(azimuthEncoder.getVoltage(), setpoint * 72));
	}

	public void setDrive(double setpoint) {

		speedMotor.set(setpoint);
	}

	public double getDrivePosition() {

		return driveEncoder.getPosition();
	}

	public void stop() {

		speedMotor.set(0);
		angleMotor.set(0);
	}

	public double getOffset() {

		return ANGLE_OFFSET;
	}
}