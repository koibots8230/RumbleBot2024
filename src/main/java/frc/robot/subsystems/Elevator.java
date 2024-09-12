package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ElevatorConstants;
import monologue.Annotations.Log;
import monologue.Logged;

// TODO change to inherit from TrapezoidProfileSubsystem. This will have an impact on how the
// simulation is done.
public class Elevator extends TrapezoidProfileSubsystem implements Logged {

  // TODO make all of these member variables final
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;

  private final AbsoluteEncoder encoder;

  private final SparkPIDController feedback;

  private final ElevatorFeedforward feedforward;

  private ElevatorSim sim;

  private PIDController simFeedback;

  private ElevatorFeedforward simFeedforward;

  private TrapezoidProfile.State setpointState;

  @Log private double setpoint;
  @Log private double position;
  @Log private double velocity;
  @Log private double leftCurrent;
  @Log private double rightCurrent;
  @Log private double appliedVoltage;

  public Elevator(boolean isReal) {
    super(
        new TrapezoidProfile.Constraints(
            ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION),
        0.0);

    leftMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_PORT, MotorType.kBrushless);

    rightMotor = new CANSparkMax(ElevatorConstants.RIGHT_MOTOR_PORT, MotorType.kBrushless);
    rightMotor.follow(leftMotor);

    encoder = leftMotor.getAbsoluteEncoder();

    feedback = leftMotor.getPIDController();

    feedback.setFeedbackDevice(encoder);

    feedback.setP(ElevatorConstants.PID_GAINS.kp);
    feedback.setI(ElevatorConstants.PID_GAINS.ki);
    feedback.setD(ElevatorConstants.PID_GAINS.kd);

    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.FEEDFORWARD_GAINS.ks,
            ElevatorConstants.FEEDFORWARD_GAINS.kg,
            ElevatorConstants.FEEDFORWARD_GAINS.kv,
            ElevatorConstants.FEEDFORWARD_GAINS.ka);
    if (!isReal) {
      sim =
          new ElevatorSim(
              DCMotor.getNEO(2),
              ElevatorConstants.GEAR_RATIO,
              2.5,
              (ElevatorConstants.DRUM_SIZE).in(Meters),
              0,
              ElevatorConstants.MAX_HEIGHT.in(Meters),
              true,
              0,
              VecBuilder.fill(0));
      simFeedback =
          new PIDController(
              ElevatorConstants.SIM_PID_GAINS.kp,
              ElevatorConstants.SIM_PID_GAINS.ki,
              ElevatorConstants.SIM_PID_GAINS.kd);
      simFeedforward =
          new ElevatorFeedforward(
              ElevatorConstants.SIM_FEEDFORWARD_GAINS.ks,
              ElevatorConstants.SIM_FEEDFORWARD_GAINS.kg,
              ElevatorConstants.SIM_FEEDFORWARD_GAINS.kv,
              ElevatorConstants.SIM_FEEDFORWARD_GAINS.ka);
    }

    setpointState = new TrapezoidProfile.State(Meters.of(0), MetersPerSecond.of(0));
  }

  @Override
  public void useState(State state) {
    setpointState = state;

    position = encoder.getPosition();
    velocity = encoder.getVelocity();

    leftCurrent = leftMotor.getOutputCurrent();
    rightCurrent = rightMotor.getOutputCurrent();

    appliedVoltage = leftMotor.getBusVoltage() * leftMotor.getAppliedOutput();

    feedback.setReference(
        setpoint, ControlType.kPosition, 0, feedforward.calculate(setpointState.velocity));
  }

  @Override
  public void simulationPeriodic() {
    sim.update(0.02);

    position = sim.getPositionMeters();

    velocity = sim.getVelocityMetersPerSecond();

    leftCurrent = sim.getCurrentDrawAmps();
    rightCurrent = sim.getCurrentDrawAmps();

    appliedVoltage =
        simFeedforward.calculate(setpointState.velocity)
            + simFeedback.calculate(position, setpoint);

    sim.setInputVoltage(appliedVoltage);
  }

  private void setPosition(Measure<Distance> position) {
    this.setGoal(position.in(Meters));
    setpoint = position.in(Meters);
  }

  public Command setPositionCommand(Measure<Distance> position) {
    return new InstantCommand(() -> this.setPosition(position), this);
  }
}
