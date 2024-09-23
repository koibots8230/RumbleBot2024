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

  private final CANSparkMax rightMotor;
  private final CANSparkMax leftMotor;

  private final SparkPIDController rightController;
  private final SparkPIDController leftController;

  private final RelativeEncoder rightEncoder;
  private final RelativeEncoder leftEncoder;

  private final DCMotorSim topSimMotor;
  private final DCMotorSim bottomSimMotor;

  private final PIDController topSimPID;
  private final PIDController bottomSimPID;

  private final SimpleMotorFeedforward topSimFeedforward;
  private final SimpleMotorFeedforward bottomSimFeedforward;

  @Log private double topSetpoint;
  @Log private double bottomSetpoint;
  @Log private double topShoterVelocity;
  @Log private double bottomShoterVelocity;
  @Log private double bottomShoterCurrent;
  @Log private double topshoterCurrent;
  @Log private double topAppliedVoltage;
  @Log private double bottomAppliedVoltage;

  // for encoder set average deapth and set average meserment piroed 2,16
  // respectivly

  public Shooter(boolean isReal) {
    // TODO change how objects get constructed here. Real objects should always be
    // constructed. Real
    // objects should have simple names that don't specify that they are real.
    // TODO Simulated objects should be constructed in the if isReal else blocks. If
    // it is
    // simulated, construct simulated objects. If it is real, simulated objects
    // should be set toz
    // null.

    rightMotor = new CANSparkMax(ShooterConstants.TOP_SHOOTER_PORT, MotorType.kBrushless);
    leftMotor = new CANSparkMax(ShooterConstants.BOTTOM_SHOOTER_PORT, MotorType.kBrushless);
    rightController = rightMotor.getPIDController();
    leftController = leftMotor.getPIDController();
    rightEncoder = rightMotor.getEncoder();
    leftEncoder = leftMotor.getEncoder();

    topSimMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
    bottomSimMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);

    topSimFeedforward = new SimpleMotorFeedforward(0.0, ShooterConstants.FEEDFORWARD_GAINS.kv);
    bottomSimFeedforward = new SimpleMotorFeedforward(0.0, ShooterConstants.FEEDFORWARD_GAINS.kv);

    topSimPID =
        new PIDController(
            ShooterConstants.PID_GAINS.kp,
            ShooterConstants.PID_GAINS.ki,
            ShooterConstants.PID_GAINS.kd);
    bottomSimPID =
        new PIDController(
            ShooterConstants.PID_GAINS.kp,
            ShooterConstants.PID_GAINS.ki,
            ShooterConstants.PID_GAINS.kd);

    if (Robot.isReal()) {
      rightMotor.restoreFactoryDefaults();
      rightMotor.setInverted(ShooterConstants.TOP_MOTOR_CONFIG.inverted);
      rightMotor.setSmartCurrentLimit(ShooterConstants.TOP_MOTOR_CONFIG.currentLimit);
      rightMotor.setIdleMode(ShooterConstants.TOP_MOTOR_CONFIG.idleMode);

      leftMotor.restoreFactoryDefaults();
      rightMotor.setSmartCurrentLimit(ShooterConstants.BOTTOM_MOTOR_CONFIG.currentLimit);
      rightMotor.setIdleMode(ShooterConstants.BOTTOM_MOTOR_CONFIG.idleMode);

      rightController.setP(ShooterConstants.PID_GAINS.kp);
      rightController.setI(ShooterConstants.PID_GAINS.ki);
      rightController.setD(ShooterConstants.PID_GAINS.kd);
      rightController.setFF(ShooterConstants.PID_GAINS.kf);

      leftController.setP(ShooterConstants.PID_GAINS.kp);
      leftController.setI(ShooterConstants.PID_GAINS.ki);
      leftController.setD(ShooterConstants.PID_GAINS.kd);
      leftController.setFF(ShooterConstants.PID_GAINS.kf);

      rightEncoder.setMeasurementPeriod(16);
      leftEncoder.setMeasurementPeriod(16);

      rightEncoder.setAverageDepth(2);
      leftEncoder.setAverageDepth(2);
    }
  }

  @Override
  public void periodic() {
    {
      rightController.setReference(topSetpoint, ControlType.kVelocity);
      leftController.setReference(bottomSetpoint, ControlType.kVelocity);
      topShoterVelocity = (rightEncoder.getVelocity());
      bottomShoterVelocity = (leftEncoder.getVelocity());
      topAppliedVoltage = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
      bottomAppliedVoltage = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
      topshoterCurrent = rightMotor.getOutputCurrent();
      bottomShoterCurrent = leftMotor.getOutputCurrent();
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

    topAppliedVoltage =
        topSimPID.calculate(topShoterVelocity, topSetpoint)
            + topSimFeedforward.calculate(topSetpoint);
    bottomAppliedVoltage =
        bottomSimPID.calculate(bottomShoterVelocity, bottomSetpoint)
            + bottomSimFeedforward.calculate(bottomSetpoint);

    topSimMotor.setInputVoltage(topAppliedVoltage);
    bottomSimMotor.setInputVoltage(bottomAppliedVoltage);
  }

  // TODO are separate velocities needed for top and bottom?
  private void setVelocity(double topSetpoint, double bottomSetpoint) {
    this.topSetpoint = topSetpoint;
    this.bottomSetpoint = bottomSetpoint;
  }

  // TODO this check should use the target set points that are passed into

  public boolean checkVelocitySpeaker() {
    return Math.abs(rightEncoder.getVelocity() - ShooterConstants.TOP_MOTOR_SETPOINT_SPEAKER)
            <= ShooterConstants.SHOOTER_RANGE
        && Math.abs(leftEncoder.getVelocity() - ShooterConstants.BOTTOM_MOTOR_SETPOINT_SPEAKER)
            <= ShooterConstants.SHOOTER_RANGE;
  }

  public boolean checkRestVelocity() {
    return Math.abs(rightEncoder.getVelocity() - ShooterConstants.REST_SETPOINT)
            <= ShooterConstants.SHOOTER_RANGE
        && Math.abs(leftEncoder.getVelocity() - ShooterConstants.REST_SETPOINT)
            <= ShooterConstants.SHOOTER_RANGE;
  }

  private void ShooterRest() {
    rightController.setReference(0.0, ControlType.kVoltage);
    leftController.setReference(0.0, ControlType.kVoltage);
  }

  public Command ShooterRestCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> ShooterRest(), this), Commands.waitUntil(() -> checkRestVelocity()));
  }

  public Command setVelocityCommand(double topSetpoint, double bottomSetpoint) {
    return Commands.sequence(
        Commands.runOnce(() -> setVelocity(topSetpoint, bottomSetpoint), this),
        Commands.waitUntil(() -> checkVelocitySpeaker()));
  }
}
