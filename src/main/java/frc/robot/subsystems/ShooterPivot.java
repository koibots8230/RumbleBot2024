package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import monologue.Annotations.Log;
import monologue.Logged;

public class ShooterPivot extends TrapezoidProfileSubsystem implements Logged {

  private final CANSparkMax shooterPivotMotor;

  private final AbsoluteEncoder encoder;

  private final SparkPIDController angPidController;

  private final ArmFeedforward armFeedforward;

  private TrapezoidProfile.State setPointState;

  @Log private double setpoint;
  @Log private double position;
  @Log private double velocity;
  @Log private double motorCurrent;
  @Log private double appliedVoltage;

  public ShooterPivot() {
    super(
        new TrapezoidProfile.Constraints(
            Constants.ShooterPivot.MAX_VELOCITY.getRadians(),
            Constants.ShooterPivot.MAX_ACCLERATION.getRadians()),
        0.0);

    shooterPivotMotor =
        new CANSparkMax(Constants.ShooterPivot.SHOOTER_PIVOT_MOTER, MotorType.kBrushless);

    encoder = shooterPivotMotor.getAbsoluteEncoder();

    angPidController = shooterPivotMotor.getPIDController();

    armFeedforward =
        new ArmFeedforward(
            Constants.ShooterPivot.FEEDFORWARD_GAINS.ks,
            Constants.ShooterPivot.FEEDFORWARD_GAINS.kg,
            Constants.ShooterPivot.FEEDFORWARD_GAINS.kv,
            Constants.ShooterPivot.FEEDFORWARD_GAINS.ka);

    angPidController.setP(Constants.ShooterPivot.PID_GAINS.kp);
    angPidController.setI(Constants.ShooterPivot.PID_GAINS.ki);
    angPidController.setD(Constants.ShooterPivot.PID_GAINS.kd);

    angPidController.setFeedbackDevice(encoder);

    setPointState = new TrapezoidProfile.State(Meters.of(0), MetersPerSecond.of(0));
  }

  @Override
  protected void useState(State state) {
    setPointState = state;

    position = encoder.getPosition();
    velocity = encoder.getVelocity();

    motorCurrent = shooterPivotMotor.getOutputCurrent();

    appliedVoltage = shooterPivotMotor.getBusVoltage() * shooterPivotMotor.getAppliedOutput();

    angPidController.setReference(
        setpoint,
        ControlType.kPosition,
        0,
        armFeedforward.calculate(setPointState.position, setPointState.velocity));
  }

  @Override
  public void simulationPeriodic() {
    position = setPointState.position;
    velocity = setPointState.velocity;
    super.simulationPeriodic();
  }

  private void setPosition(Rotation2d position) {
    this.setGoal(position.getRadians());
    setpoint = position.getRadians();
  }

  public double linnerInterpilation(double distance) {
    double angle = Constants.ShooterPivot.a * distance + Constants.ShooterPivot.b;
    return angle;
  }

  public void initDefaultCommand() {
    setDefaultCommand(
        new InstantCommand(() -> this.setPosition(Constants.ShooterPivot.REST_POSITION)));
  }

  // to do make this part of the shooting process
  public Command setPositionCommand(Rotation2d position) {
    return new InstantCommand(() -> this.setPosition(position), this);
  }
  public void setRestPosition(Rotation2d position) {
    setPosition(position);
  }
  public Command setRestPositionCommand(Rotation2d position){
    return Commands.run(() -> setRestPosition(position), this);
  }

}
