package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
            Constants.ShooterPivotConstants.MAX_VELOCITY.getRadians(),
            Constants.ShooterPivotConstants.MAX_ACCLERATION.getRadians()),
        0.0);

    shooterPivotMotor =
        new CANSparkMax(Constants.ShooterPivotConstants.SHOOTER_PIVOT_MOTER, MotorType.kBrushless);

    encoder = shooterPivotMotor.getAbsoluteEncoder();

    angPidController = shooterPivotMotor.getPIDController();

    armFeedforward =
        new ArmFeedforward(
            Constants.ShooterPivotConstants.FEEDFORWARD_GAINS.ks,
            Constants.ShooterPivotConstants.FEEDFORWARD_GAINS.kg,
            Constants.ShooterPivotConstants.FEEDFORWARD_GAINS.kv,
            Constants.ShooterPivotConstants.FEEDFORWARD_GAINS.ka);

    angPidController.setP(Constants.ShooterPivotConstants.PID_GAINS.kp);
    angPidController.setI(Constants.ShooterPivotConstants.PID_GAINS.ki);
    angPidController.setD(Constants.ShooterPivotConstants.PID_GAINS.kd);

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

  public Rotation2d getEstimatedAngle(double distance) {
    if (distance < 1.5){
      double angle =
              Constants.ShooterPivotConstants.AUTO_ANGLE_SLOPE_POSITION_ONE * distance
                      + Constants.ShooterPivotConstants.Y_INTERCEPT_POSITON_ONE;
      return Rotation2d.fromDegrees(angle);
    }
    if(distance < 2.5){
      double angle =
              Constants.ShooterPivotConstants.AUTO_ANGLE_SLOPE_POSITION_TWO * distance
                      + Constants.ShooterPivotConstants.Y_INTERCEPT_POSITON_TWO;
      return Rotation2d.fromDegrees(angle);
    }
    if(distance < 3.5){
      double angle =
              Constants.ShooterPivotConstants.AUTO_ANGLE_SLOPE_POSITION_THREE * distance
                      + Constants.ShooterPivotConstants.Y_INTERCEPT_POSITON_THREE;
      return Rotation2d.fromDegrees(angle);
    }
    if(distance < 4.5){
      double angle =
              Constants.ShooterPivotConstants.AUTO_ANGLE_SLOPE_POSITION_FOUR * distance
                      + Constants.ShooterPivotConstants.Y_INTERCEPT_POSITON_FOUR;
      return Rotation2d.fromDegrees(angle);
    }
    if(distance > 4.5){
      double angle =
              Constants.ShooterPivotConstants.AUTO_ANGLE_SLOPE_POSITION_FIVE * distance
                      + Constants.ShooterPivotConstants.Y_INTERCEPT_POSITON_FIVE;
      return Rotation2d.fromDegrees(angle);
    }
    return new Rotation2d(0);
  }

  public double getSpeakerDistance(Pose2d robotPose) {
    if (DriverStation.getAlliance().isPresent()) {
      Pose2d speakerPose =
          DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
              ? Constants.FieldConstants.BLUE_SPEAKER_POSE
              : Constants.FieldConstants.RED_SPEAKER_POSE;
      double speakerDistance =
          Math.hypot(
              (robotPose.getX() - speakerPose.getX()), (robotPose.getY() - speakerPose.getY()));
      return speakerDistance;
    }
    return 0.0;
  }

  // to do make this part of the shooting process
  public Command setPositionCommand(Rotation2d position) {
    return Commands.runOnce(() -> this.setPosition(position), this);
  }

  public Command autoSetAngle(Pose2d tempval) {
    return Commands.runOnce(
        () -> setPosition(getEstimatedAngle(getSpeakerDistance(tempval))), this);
  }
}
