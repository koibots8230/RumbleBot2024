package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class SwerveModule implements Logged {
  private final CANSparkMax turnMotor;
  private final AbsoluteEncoder turnEncoder;

  private final SparkPIDController turnPID;
  private final SimpleMotorFeedforward turnFF;

  private final TrapezoidProfile turnProfile;
  private TrapezoidProfile.State turnGoalState;
  private TrapezoidProfile.State turnSetpointState;

  private final CANSparkFlex driveMotor;
  private final RelativeEncoder driveEncoder;

  private final SparkPIDController drivePID;

  private final Rotation2d angleOffset;

  @Log private Rotation2d turnSetpoint;
  @Log private Rotation2d turnPosition;
  @Log private double turnVelocity;

  @Log private double turnCurrent;
  @Log private double turnVoltage;

  @Log private double driveSetpoint;
  @Log private double drivePosition;
  @Log private double driveVelocity;

  @Log private double driveCurrent;
  @Log private double driveVoltage;

  public SwerveModule(int turnID, int driveID, Rotation2d angleOffset) {
    turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

    turnMotor.restoreFactoryDefaults();

    turnMotor.setSmartCurrentLimit(SwerveConstants.TURN_MOTOR_CONFIG.currentLimit);
    turnMotor.setInverted(SwerveConstants.TURN_MOTOR_CONFIG.inverted);
    turnMotor.setIdleMode(SwerveConstants.TURN_MOTOR_CONFIG.idleMode);
    turnMotor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));
    turnMotor.setCANTimeout((int) RobotConstants.CAN_TIMEOUT.in(Milliseconds));

    turnEncoder = turnMotor.getAbsoluteEncoder();

    turnEncoder.setPositionConversionFactor(
        SwerveConstants.TURN_ENCODER_POSITION_FACTOR.in(Radians));
    turnEncoder.setVelocityConversionFactor(
        SwerveConstants.TURN_ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

    turnEncoder.setInverted(true);

    turnPID = turnMotor.getPIDController();
    turnPID.setFeedbackDevice(turnEncoder);

    turnPID.setP(SwerveConstants.TURN_PID_GAINS.kp);

    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMaxInput(Math.PI);
    turnPID.setPositionPIDWrappingMinInput(-Math.PI);

    turnFF =
        new SimpleMotorFeedforward(
            SwerveConstants.TURN_FF_GAINS.ks, SwerveConstants.TURN_FF_GAINS.kv);

    turnProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                SwerveConstants.MAX_TURN_VELOCITY.in(RadiansPerSecond),
                SwerveConstants.MAX_TURN_ACCELERATION.in(RadiansPerSecond.per(Second))));

    turnGoalState = new TrapezoidProfile.State(0, 0);
    turnSetpointState = new TrapezoidProfile.State(0, 0);

    driveMotor = new CANSparkFlex(driveID, MotorType.kBrushless);

    driveMotor.setSmartCurrentLimit(SwerveConstants.DRIVE_MOTOR_CONFIG.currentLimit);
    driveMotor.setInverted(SwerveConstants.DRIVE_MOTOR_CONFIG.inverted);
    driveMotor.setIdleMode(SwerveConstants.DRIVE_MOTOR_CONFIG.idleMode);
    driveMotor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));
    driveMotor.setCANTimeout((int) RobotConstants.CAN_TIMEOUT.in(Milliseconds));

    driveEncoder = driveMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(
        SwerveConstants.DRIVE_ENCODER_POSITION_FACTOR.in(Meters));
    driveEncoder.setVelocityConversionFactor(
        SwerveConstants.DRIVE_ENCODER_VELOCITY_FACTOR.in(MetersPerSecond));

    driveEncoder.setPosition(0);
    driveEncoder.setAverageDepth(2);
    driveEncoder.setMeasurementPeriod(8);

    drivePID = driveMotor.getPIDController();
    drivePID.setFeedbackDevice(driveEncoder);

    drivePID.setP(SwerveConstants.DRIVE_PID_GAINS.kp);
    drivePID.setFF(SwerveConstants.DRIVE_FF_GAINS.kv);

    turnSetpoint = Rotation2d.fromRadians(0);
    turnPosition = Rotation2d.fromRadians(0);

    driveSetpoint = 0;
    driveVelocity = 0;

    this.angleOffset = angleOffset;
  }

  public void setState(SwerveModuleState state) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, turnPosition);

    optimizedState.speedMetersPerSecond =
        optimizedState.speedMetersPerSecond * optimizedState.angle.minus(turnPosition).getCos();

    driveSetpoint = optimizedState.speedMetersPerSecond;

    turnGoalState =
        new TrapezoidProfile.State(optimizedState.angle.getRadians() + angleOffset.getRadians(), 0);

    turnSetpointState = turnProfile.calculate(0.02, turnSetpointState, turnGoalState);

    turnPID.setReference(
        optimizedState.angle.getRadians(),
        ControlType.kPosition,
        0,
        turnFF.calculate(turnSetpointState.velocity));

    drivePID.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
  }

  public void periodic() {
    turnSetpoint = Rotation2d.fromRadians(turnGoalState.position);

    turnPosition = Rotation2d.fromRadians(turnEncoder.getPosition() + angleOffset.getRadians());
    turnVelocity = turnEncoder.getVelocity();

    turnCurrent = turnMotor.getOutputCurrent();
    turnVoltage = turnMotor.getBusVoltage() * turnMotor.getAppliedOutput();

    drivePosition = driveEncoder.getPosition();
    driveVelocity = driveEncoder.getVelocity();

    driveCurrent = driveMotor.getOutputCurrent();
    driveVoltage = driveMotor.getBusVoltage() * driveMotor.getAppliedOutput();
  }

  public void simulationPeriodic() {
    turnPosition = Rotation2d.fromRadians(turnSetpointState.position);
    driveVelocity = driveSetpoint;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveVelocity, turnPosition);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(drivePosition, turnPosition);
  }
}
