package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class Manipulator extends SubsystemBase {

    private final CANSparkMax armNEO;
    private final CANSparkMax armNEOFollower;
    private final CANSparkMax slideNEO;

    private final RelativeEncoder armThroughBore;

    private final SparkMaxPIDController slidePIDController;

    private final ArmFeedforward armFF;

    private final DigitalInput armZeroLimit;

    private final ProfiledPIDController armController;

    private TrapezoidProfile.State armGoalState = 
        new TrapezoidProfile.State();

    private TrapezoidProfile.State slideGoalState = 
        new TrapezoidProfile.State();

    static Manipulator manipInstance;


    public Manipulator() {

        armNEO             = new CANSparkMax(50, MotorType.kBrushless);
        armNEOFollower     = new CANSparkMax(51, MotorType.kBrushless);
        slideNEO           = new CANSparkMax(40, MotorType.kBrushless);

        armThroughBore     = armNEO.getAlternateEncoder(8192);

        slidePIDController = slideNEO.getPIDController();

        armZeroLimit       = new DigitalInput(0);

        armFF              = new ArmFeedforward(0.74, 0.25, 3.9);

        armController      = new ProfiledPIDController(
                                1, 0.0, 0.0, 
                                new TrapezoidProfile.Constraints(12, 12),
                                0.02
                            );

        armThroughBore.setPositionConversionFactor(6.28);
        armThroughBore.setVelocityConversionFactor(0.105);

        slideNEO.enableVoltageCompensation(12);
        slideNEO.setSmartCurrentLimit(40);
        slideNEO.setIdleMode(IdleMode.kBrake);
        slidePIDController.setP(0.11);
        slidePIDController.setI(0);
        slidePIDController.setD(0);
        slidePIDController.setOutputRange(-1, 1);

        armNEO.burnFlash();
        slideNEO.burnFlash();

        armNEOFollower.follow(armNEO, true);

        setArmGoal(armGoalState);
        setSlideGoal(slideGoalState);
    }


    public void setArmGoal(State goal) {
        armController.setGoal(goal);
        armGoalState = goal;
    }

    public void setSlideGoal(State goal) {
        slideGoalState = goal;
    }

    public void resetController() {
        armController.reset(getArmEncoderCounts());
    }

    public boolean atSlideSetpoint() {
        return Math.abs(getSlideEncoderCounts() - slideGoalState.position) < 0.02;
    }

    public double getArmEncoderCounts() {
        return armThroughBore.getPosition();
    }

    public double getArmEncoderVelocity() {
        return armThroughBore.getVelocity();
    }

    public double getSlideEncoderCounts() {
        return slideNEO.getEncoder().getPosition();
    }

    public void setArmAngle(double angle) {
        armNEO.set(angle + armFF.calculate((angle - 1.48) + 0.01, 0));
    }

    public void setSlideDistance(double distance) {
        slidePIDController.setReference(distance, ControlType.kPosition);
    }

    public boolean atArmZeroLimit() {
        return !armZeroLimit.get();
    }

    public void setManipGoal(Translation2d goalTranslation) {
        setArmGoal(new TrapezoidProfile.State(goalTranslation.getAngle().getRadians(), 0));
        setSlideGoal(new TrapezoidProfile.State(goalTranslation.getNorm(), 0));
    }

    
    public void periodic() {

        SmartDashboard.putNumber("Current Arm Angle", getArmEncoderCounts());
        SmartDashboard.putNumber("Current Slide Length", getSlideEncoderCounts());

        SmartDashboard.putNumber("Goal Arm Angle", armGoalState.position);
        SmartDashboard.putNumber("Goal Slide Length", slideGoalState.position);

        //setSlideDistance(slideGoalState.position);

        if(atArmZeroLimit())   {armThroughBore.setPosition(0);}

        double theta = armController.calculate(getArmEncoderCounts());

        if(theta < -0.1) {theta = -0.1;}
        
        if (armController.atSetpoint() && (getSlideEncoderCounts() >= 10) ) {
            setArmAngle(getSlideEncoderCounts() / 3600);
        } else {setArmAngle(theta);}
    }

    public static Manipulator getInstance() {
        if(manipInstance == null) {
            manipInstance = new Manipulator();
        } return manipInstance;
    }
}