package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import monologue.Annotations.Log;
import monologue.Logged;

public class Shooter extends SubsystemBase implements Logged {

  private final CANSparkMax topMotor;
  private final CANSparkMax bottomMotor;

  private final SparkPIDController topController;
  private final SparkPIDController bottomController;

  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;

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
    // TODO change how objects get constructed here. Real objects should always be constructed. Real
    // objects should have simple names that don't specify that they are real.
    // TODO Simulated objects should be constructed in the if isReal else blocks. If it is
    // simulated, construct simulated objects. If it is real, simulated objects should be set toz
    // null.


    topMotor = new CANSparkMax(ShooterConstants.TOP_SHOOTER_PORT, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(ShooterConstants.BOTTOM_SHOOTER_PORT, MotorType.kBrushless);
    topController = topMotor.getPIDController();
    bottomController = bottomMotor.getPIDController();
    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();

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
      topMotor.restoreFactoryDefaults();
      topMotor.setInverted(ShooterConstants.TOP_MOTOR_CONFIG.inverted);
      topMotor.setSmartCurrentLimit(ShooterConstants.TOP_MOTOR_CONFIG.currentLimit);
      topMotor.setIdleMode(ShooterConstants.TOP_MOTOR_CONFIG.idleMode);

      bottomMotor.restoreFactoryDefaults();
      topMotor.setSmartCurrentLimit(ShooterConstants.BOTTOM_MOTOR_CONFIG.currentLimit);
      topMotor.setIdleMode(ShooterConstants.BOTTOM_MOTOR_CONFIG.idleMode);

      topController.setP(ShooterConstants.PID_GAINS.kp);
      topController.setI(ShooterConstants.PID_GAINS.ki);
      topController.setD(ShooterConstants.PID_GAINS.kd);
      topController.setFF(ShooterConstants.PID_GAINS.kf);

      bottomController.setP(ShooterConstants.PID_GAINS.kp);
      bottomController.setI(ShooterConstants.PID_GAINS.ki);
      bottomController.setD(ShooterConstants.PID_GAINS.kd);
      bottomController.setFF(ShooterConstants.PID_GAINS.kf);

      topEncoder.setMeasurementPeriod(16);
      bottomEncoder.setMeasurementPeriod(16);

      topEncoder.setAverageDepth(2);
      bottomEncoder.setAverageDepth(2);
    }
  }

  @Override
  public void periodic() {
    // TODO Periodic is always called on real objects. Remove the "if block" around these calls.
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
  public void setVelocity(double topSetpoint, double bottomSetpoint) {
    this.topSetpoint = topSetpoint;
    this.bottomSetpoint = bottomSetpoint;
  }

  // TODO this check should use the target set points that are passed into setVelocity, not from the
  // constants file.
  public boolean checkVelocity() {
    return Math.abs(topEncoder.getVelocity() - ShooterConstants.TOP_MOTOR_SETPOINT_APM.in(RPM))
                <= ShooterConstants.SHOOTER_RANGE
            && Math.abs(
                    bottomEncoder.getVelocity()
                        - ShooterConstants.BOTTOM_MOTOR_SEETPOINT_APM.in(RPM))
                <= ShooterConstants.SHOOTER_RANGE
        || Math.abs(topEncoder.getVelocity() - ShooterConstants.TOP_MOTOR_SETPOINT_SPEAKER.in(RPM))
                <= ShooterConstants.SHOOTER_RANGE
            && Math.abs(
                    bottomEncoder.getVelocity()
                        - ShooterConstants.BOTTOM_MOTOR_SETPOINT_SPEAKER.in(RPM))
                <= ShooterConstants.SHOOTER_RANGE;
  }
}
