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
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Elevator extends SubsystemBase implements Logged {

  private boolean isReal;

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private ElevatorSim sim;

  private AbsoluteEncoder encoder;

  private SparkPIDController realFeedback;

  private PIDController simFeedback;

  private ElevatorFeedforward feedforward;

  private TrapezoidProfile profile;
  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State setpointState;

  @Log private double setpoint;
  @Log private double position;
  @Log private double velocity;
  @Log private double leftCurrent;
  @Log private double rightCurrent;
  @Log private double appliedVoltage;

  public Elevator(boolean isReal) {
    this.isReal = isReal;

    if (isReal) {
      leftMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_PORT, MotorType.kBrushless);

      rightMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_PORT, MotorType.kBrushless);
      rightMotor.follow(leftMotor);

      encoder = leftMotor.getAbsoluteEncoder();

      realFeedback = leftMotor.getPIDController();

      realFeedback.setFeedbackDevice(encoder);

      realFeedback.setP(ElevatorConstants.REAL_FEEDBACK_CONSTANTS.kp);
      realFeedback.setI(ElevatorConstants.REAL_FEEDBACK_CONSTANTS.ki);
      realFeedback.setD(ElevatorConstants.REAL_FEEDBACK_CONSTANTS.kd);

      feedforward =
          new ElevatorFeedforward(
              ElevatorConstants.REAL_FEEDFORWARD_CONSTANTS.ks,
              ElevatorConstants.REAL_FEEDFORWARD_CONSTANTS.kg,
              ElevatorConstants.REAL_FEEDFORWARD_CONSTANTS.kv,
              ElevatorConstants.REAL_FEEDFORWARD_CONSTANTS.ka);
    } else {
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
              ElevatorConstants.SIM_FEEDBACK_CONSTANTS.kp,
              ElevatorConstants.SIM_FEEDBACK_CONSTANTS.ki,
              ElevatorConstants.SIM_FEEDBACK_CONSTANTS.kd);
      feedforward =
          new ElevatorFeedforward(
              ElevatorConstants.SIM_FEEDFORWARD_CONSTANTS.ks,
              ElevatorConstants.SIM_FEEDFORWARD_CONSTANTS.kg,
              ElevatorConstants.SIM_FEEDFORWARD_CONSTANTS.kv,
              ElevatorConstants.SIM_FEEDFORWARD_CONSTANTS.ka);
    }

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION));
    goal = new TrapezoidProfile.State(Meters.of(0), MetersPerSecond.of(0));
    setpointState = new TrapezoidProfile.State(Meters.of(0), MetersPerSecond.of(0));
  }

  @Override
  public void periodic() {
    setpointState = profile.calculate(0.02, setpointState, goal);
    setpoint = goal.position;
    if (isReal) {
      position = encoder.getPosition();
      velocity = encoder.getVelocity();

      leftCurrent = leftMotor.getOutputCurrent();
      rightCurrent = rightMotor.getOutputCurrent();

      appliedVoltage = leftMotor.getBusVoltage() * leftMotor.getAppliedOutput();

      realFeedback.setReference(
          setpoint, ControlType.kPosition, 0, feedforward.calculate(setpointState.velocity));
    }
  }

  @Override
  public void simulationPeriodic() {
    sim.update(0.02);

    position = sim.getPositionMeters();

    velocity = sim.getVelocityMetersPerSecond();

    leftCurrent = sim.getCurrentDrawAmps();
    rightCurrent = sim.getCurrentDrawAmps();

    appliedVoltage =
        feedforward.calculate(setpointState.velocity) + simFeedback.calculate(position, setpoint);

    sim.setInputVoltage(appliedVoltage);
  }

  private void setGoal(Measure<Distance> position) {
    goal.position = position.in(Meters);
  }

  public Command setPosition(Measure<Distance> position) {
    return new InstantCommand(() -> this.setGoal(position));
  }
}
