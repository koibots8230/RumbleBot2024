package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class Shooter extends SubsystemBase implements Logged {

    private CANSparkMax topMotor;
    private CANSparkMax bottomMotor;

    private SparkPIDController topController;
    private SparkPIDController bottomController;

    private RelativeEncoder topEncoder;
    private RelativeEncoder bottomEncoder;

    private DCMotorSim topSimMotor;
    private DCMotorSim bottomSimMotor;

    private PIDController topSimPID;
    private PIDController bottomSimPID;

    private SimpleMotorFeedforward topSimFeedforward;
    private SimpleMotorFeedforward bottomSimFeedforward;

    boolean isReal;

    @Log
    private double topSetpoint;
    @Log
    private double bottomSetpoint;
    @Log
    private double topShoterVelocity;
    @Log
    private double bottomShoterVelocity;
    @Log
    private double bottomShoterCurrent;
    @Log
    private double topshoterCurrent;
    @Log
    private double topAppliedVoltage;
    @Log
    private double bottomAppliedVoltage;

    // for encoder set average deapth and set average meserment piroed 2,16
    // respectivly

    public Shooter(boolean isReal) {
        this.isReal = isReal;
        if (Robot.isReal()) {
            topMotor = new CANSparkMax(ShooterConstants.TOP_SHOOTER_PORT, MotorType.kBrushless);
            topMotor.restoreFactoryDefaults();
            topMotor.setInverted(ShooterConstants.TOP_MOTOR_CONSTANTS_IO.inverted);
            topMotor.setSmartCurrentLimit(ShooterConstants.TOP_MOTOR_CONSTANTS_IO.currentLimit);
            topMotor.setIdleMode(ShooterConstants.TOP_MOTOR_CONSTANTS_IO.idleMode);

            bottomMotor = new CANSparkMax(ShooterConstants.BOTTOM_SHOOTER_PORT, MotorType.kBrushless);
            bottomMotor.restoreFactoryDefaults();
            topMotor.setSmartCurrentLimit(ShooterConstants.BOTTOM_MOTOR_CONSTANTS_IO.currentLimit);
            topMotor.setIdleMode(ShooterConstants.BOTTOM_MOTOR_CONSTANTS_IO.idleMode);

            topController = topMotor.getPIDController();

            topController.setP(ShooterConstants.TOP_PID_CONSTANTS_IO.kP);
            topController.setI(ShooterConstants.TOP_PID_CONSTANTS_IO.kI);
            topController.setD(ShooterConstants.TOP_PID_CONSTANTS_IO.kD);
            topController.setFF(ShooterConstants.TOP_FEEDFORWARD_CONSTANTS_IO.kv);

            bottomController = bottomMotor.getPIDController();

            bottomController.setP(ShooterConstants.BOTTOM_PID_CONSTANTS_IO.kP);
            bottomController.setI(ShooterConstants.BOTTOM_PID_CONSTANTS_IO.kI);
            bottomController.setD(ShooterConstants.BOTTOM_PID_CONSTANTS_IO.kD);
            bottomController.setFF(ShooterConstants.BOTTOM_FEEDFORWARD_CONSTANTS_IO.kv);

            topEncoder = topMotor.getEncoder();
            bottomEncoder = bottomMotor.getEncoder();

            topEncoder.setMeasurementPeriod(16);
            bottomEncoder.setMeasurementPeriod(16);

            topEncoder.setAverageDepth(2);
            bottomEncoder.setAverageDepth(2);
        } else {

            topSimMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
            bottomSimMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);

            topSimFeedforward = new SimpleMotorFeedforward(0.0, ShooterConstants.TOP_FEEDFORWARD_CONSTANTS_IO.kv);
            bottomSimFeedforward = new SimpleMotorFeedforward(0.0, ShooterConstants.BOTTOM_FEEDFORWARD_CONSTANTS_IO.kv);

            topSimPID = new PIDController(ShooterConstants.TOP_PID_CONSTANTS_IO.kP, 0.0, 0.00);
            bottomSimPID = new PIDController(ShooterConstants.BOTTOM_PID_CONSTANTS_IO.kP, 0.0, 0.00);


        }
    }

    @Override
    public void periodic() {
        if (Robot.isReal()) {
            topController.setReference(topSetpoint, ControlType.kVelocity);
            bottomController.setReference(bottomSetpoint, ControlType.kVelocity);
            topShoterVelocity = topEncoder.getVelocity();
            bottomShoterVelocity = bottomEncoder.getVelocity();
            topAppliedVoltage = topMotor.getAppliedOutput() * topMotor.getBusVoltage();
            bottomAppliedVoltage = bottomMotor.getAppliedOutput() * bottomMotor.getBusVoltage();
            topshoterCurrent = topMotor.getOutputCurrent();
            bottomShoterCurrent = bottomMotor.getOutputCurrent();
        }
    }

    @Override
    public void simulationPeriodic() {
        topSimMotor.update(.02);
        bottomSimMotor.update(.02);

        topshoterCurrent = topSimMotor.getCurrentDrawAmps();
        topShoterVelocity = topSimMotor.getAngularVelocityRPM();

        bottomShoterVelocity = bottomSimMotor.getAngularVelocityRPM();
        bottomShoterCurrent = bottomSimMotor.getCurrentDrawAmps();

        topAppliedVoltage = topSimPID.calculate(topShoterVelocity, topSetpoint)
                + topSimFeedforward.calculate(topSetpoint);
        bottomAppliedVoltage = bottomSimPID.calculate(bottomShoterVelocity, bottomSetpoint)
                + bottomSimFeedforward.calculate(bottomSetpoint);

        topSimMotor.setInputVoltage(topAppliedVoltage);
        bottomSimMotor.setInputVoltage(bottomAppliedVoltage);
    }

    public void setVelocity(double topSetpoint, double bottomSetpoint) {
        this.topSetpoint = topSetpoint;
        this.bottomSetpoint = bottomSetpoint;
    }

    public boolean checkVelocity() {
        return Math
                .abs(topEncoder.getVelocity()
                        - ShooterConstants.TOP_MOTOR_SETPOINT_APM.in(RPM)) <= ShooterConstants.SHOOTER_RANGE
                &&
                Math.abs(bottomEncoder.getVelocity()
                        - ShooterConstants.BOTTOM_MOTOR_SEETPOINT_APM.in(RPM)) <= ShooterConstants.SHOOTER_RANGE
                ||
                Math.abs(topEncoder.getVelocity()
                        - ShooterConstants.TOP_MOTOR_SETPOINT_SPEAKER.in(RPM)) <= ShooterConstants.SHOOTER_RANGE
                        &&
                        Math.abs(bottomEncoder.getVelocity()
                                - ShooterConstants.BOTTOM_MOTOR_SETPOINT_SPEAKER
                                        .in(RPM)) <= ShooterConstants.SHOOTER_RANGE;
    }

}
