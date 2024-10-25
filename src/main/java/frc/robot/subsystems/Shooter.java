package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import monologue.Annotations.Log;
import monologue.Logged;

public class Shooter extends SubsystemBase implements Logged {

  // FIX THE NAMES

  private final CANSparkMax rightMotor;
  private final CANSparkMax leftMotor;

  private final SparkPIDController rightController;
  private final SparkPIDController leftController;

  private final RelativeEncoder rightEncoder;
  private final RelativeEncoder leftEncoder;

  private final DCMotorSim rightSimMotor;
  private final DCMotorSim leftSimMotor;

  private final PIDController rightSimPID;
  private final PIDController leftSimPID;

  private final SimpleMotorFeedforward rightSimFeedforward;
  private final SimpleMotorFeedforward leftSimFeedforward;

  @Log private double rightSetpoint;
  @Log private double leftSetpoint;
  @Log private double rightShoterVelocity;
  @Log private double leftShoterVelocity;
  @Log private double leftShoterCurrent;
  @Log private double rightshoterCurrent;
  @Log private double rightAppliedVoltage;
  @Log private double leftAppliedVoltage;

  private boolean shooterAtRest;

  public Shooter(boolean isReal) {

    shooterAtRest = false;

    rightMotor = new CANSparkMax(ShooterConstants.RIGHT_SHOOTER_PORT, MotorType.kBrushless);
    leftMotor = new CANSparkMax(ShooterConstants.LEFT_SHOOTER_PORT, MotorType.kBrushless);
    rightController = rightMotor.getPIDController();
    leftController = leftMotor.getPIDController();
    rightEncoder = rightMotor.getEncoder();
    leftEncoder = leftMotor.getEncoder();

    rightSimMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
    leftSimMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);

    rightSimFeedforward = new SimpleMotorFeedforward(0.0, ShooterConstants.FEEDFORWARD_GAINS.kv);
    leftSimFeedforward = new SimpleMotorFeedforward(0.0, ShooterConstants.FEEDFORWARD_GAINS.kv);

    rightSimPID =
        new PIDController(
            ShooterConstants.PID_GAINS.kp,
            ShooterConstants.PID_GAINS.ki,
            ShooterConstants.PID_GAINS.kd);
    leftSimPID =
        new PIDController(
            ShooterConstants.PID_GAINS.kp,
            ShooterConstants.PID_GAINS.ki,
            ShooterConstants.PID_GAINS.kd);

    if (Robot.isReal()) {
      rightMotor.restoreFactoryDefaults();
      rightMotor.setInverted(ShooterConstants.RIGHT_MOTOR_CONFIG.inverted);
      rightMotor.setSmartCurrentLimit(ShooterConstants.RIGHT_MOTOR_CONFIG.currentLimit);
      rightMotor.setIdleMode(ShooterConstants.RIGHT_MOTOR_CONFIG.idleMode);

      leftMotor.restoreFactoryDefaults();
      leftMotor.setSmartCurrentLimit(ShooterConstants.LEFT_MOTOR_CONFIG.currentLimit);
      leftMotor.setIdleMode(ShooterConstants.LEFT_MOTOR_CONFIG.idleMode);

      rightController.setP(ShooterConstants.PID_GAINS.kp);
      rightController.setI(ShooterConstants.PID_GAINS.ki);
      rightController.setD(ShooterConstants.PID_GAINS.kd);
      rightController.setFF(ShooterConstants.FEEDFORWARD_GAINS.kv);

      leftController.setP(ShooterConstants.PID_GAINS.kp);
      leftController.setI(ShooterConstants.PID_GAINS.ki);
      leftController.setD(ShooterConstants.PID_GAINS.kd);
      leftController.setFF(ShooterConstants.FEEDFORWARD_GAINS.kv);

      rightEncoder.setMeasurementPeriod(16);
      leftEncoder.setMeasurementPeriod(16);

      rightEncoder.setAverageDepth(2);
      leftEncoder.setAverageDepth(2);
    }
  }

  @Override
  public void periodic() {
    {
      System.out.println(shooterAtRest);
      if (!shooterAtRest) {
        rightController.setReference(rightSetpoint, ControlType.kVelocity);
        leftController.setReference(leftSetpoint, ControlType.kVelocity);
      }
      rightShoterVelocity = (rightEncoder.getVelocity());
      leftShoterVelocity = (leftEncoder.getVelocity());
      rightAppliedVoltage = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
      leftAppliedVoltage = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
      rightshoterCurrent = rightMotor.getOutputCurrent();
      leftShoterCurrent = leftMotor.getOutputCurrent();
    }
  }

  @Override
  public void simulationPeriodic() {
    rightSimMotor.update(.02);
    leftSimMotor.update(.02);

    rightshoterCurrent = rightSimMotor.getCurrentDrawAmps();
    rightShoterVelocity = rightSimMotor.getAngularVelocityRPM();

    leftShoterVelocity = leftSimMotor.getAngularVelocityRPM();
    leftShoterCurrent = leftSimMotor.getCurrentDrawAmps();

    rightAppliedVoltage =
        rightSimPID.calculate(rightShoterVelocity, rightSetpoint)
            + rightSimFeedforward.calculate(rightSetpoint);
    leftAppliedVoltage =
        leftSimPID.calculate(leftShoterVelocity, leftSetpoint)
            + leftSimFeedforward.calculate(leftSetpoint);

    rightSimMotor.setInputVoltage(rightAppliedVoltage);
    leftSimMotor.setInputVoltage(leftAppliedVoltage);
  }

  // TODO are separate velocities needed for top and bottom?
  private void setVelocity(double rightSetpoint, double leftSetpoint) {
    shooterAtRest = false;
    this.rightSetpoint = rightSetpoint;
    this.leftSetpoint = leftSetpoint;
  }

  // TODO this check should use the target set points that are passed into

  public boolean checkVelocitySpeaker() {
    return Math.abs(rightEncoder.getVelocity() - ShooterConstants.RIGHT_MOTOR_SETPOINT_SPEAKER)
            <= ShooterConstants.SHOOTER_RANGE
        && Math.abs(leftEncoder.getVelocity() - ShooterConstants.LEFT_MOTOR_SETPOINT_SPEAKER)
            <= ShooterConstants.SHOOTER_RANGE;
  }

  public boolean checkRestVelocity() {
    return Math.abs(rightEncoder.getVelocity() - ShooterConstants.REST_SETPOINT)
            <= ShooterConstants.SHOOTER_RANGE
        && Math.abs(leftEncoder.getVelocity() - ShooterConstants.REST_SETPOINT)
            <= ShooterConstants.SHOOTER_RANGE;
  }

  private void ShooterRest() {
    shooterAtRest = true;
    rightSetpoint = 0.0;
    leftSetpoint = 0.0;
    rightController.setReference(0.0, ControlType.kVoltage);
    leftController.setReference(0.0, ControlType.kVoltage);
  }

  public Command ShooterRestCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> ShooterRest(), this), Commands.waitUntil(() -> checkRestVelocity()));
  }

  public Command setVelocityCommand(double rightSetpoint, double leftSetpoint) {
    return Commands.sequence(
        Commands.runOnce(() -> setVelocity(rightSetpoint, leftSetpoint), this),
        Commands.waitUntil(() -> checkVelocitySpeaker()));
  }
}
