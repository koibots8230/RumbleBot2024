package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Indexer extends SubsystemBase implements Logged {

  private final CANSparkMax topMotor;
  private final CANSparkMax bottomMotor;

  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;

  private final SparkPIDController topController;
  private final SparkPIDController bottomController;

  private final DigitalInput topNoteDetector;
  private final DigitalInput bottomNoteDetector;

  @Log private double topSetpoint;
  @Log private double topVelocity;
  @Log private double topCurrent;
  @Log private double topAppliedVoltage;

  @Log private double bottomSetpoint;
  @Log private double bottomVelocity;
  @Log private double bottomCurrent;
  @Log private double bottomAppliedVoltage;

  private boolean hasNote;

  public Indexer() {
    topMotor =
        new CANSparkMax(IndexerConstants.TOP_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless);
    bottomMotor =
        new CANSparkMax(IndexerConstants.BOTTOM_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless);

    topMotor.setSmartCurrentLimit(IndexerConstants.TOP_MOTOR_CONFIG.currentLimit);
    topMotor.setIdleMode(IndexerConstants.TOP_MOTOR_CONFIG.idleMode);
    topMotor.setInverted(IndexerConstants.TOP_MOTOR_CONFIG.inverted);

    bottomMotor.setSmartCurrentLimit(IndexerConstants.BOTTOM_MOTOR_CONFIG.currentLimit);
    bottomMotor.setIdleMode(IndexerConstants.BOTTOM_MOTOR_CONFIG.idleMode);
    bottomMotor.setInverted(IndexerConstants.BOTTOM_MOTOR_CONFIG.inverted);

    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();

    topEncoder.setVelocityConversionFactor(IndexerConstants.TOP_GEAR_RATIO);
    bottomEncoder.setVelocityConversionFactor(IndexerConstants.BOTTOM_GEAR_RATIO);

    topController = topMotor.getPIDController();

    topController.setP(IndexerConstants.TOP_FEEDBACK_GAINS.kp);
    topController.setFF(IndexerConstants.TOP_FEEDFORWARD_GAINS.kv);

    bottomController = bottomMotor.getPIDController();

    bottomController.setP(IndexerConstants.BOTTOM_FEEDBACK_GAINS.kp);
    bottomController.setFF(IndexerConstants.BOTTOM_FEEDFORWARD_GAINS.kv);

    topNoteDetector = new DigitalInput(IndexerConstants.TOP_NOTE_DETECTOR_PORT);
    bottomNoteDetector = new DigitalInput(IndexerConstants.BOTTOM_NOTE_DETECTOR_PORT);
  }

  @Override
  public void periodic() {
    topVelocity = topMotor.getEncoder().getVelocity();
    bottomVelocity = bottomMotor.getEncoder().getVelocity();

    topCurrent = topMotor.getOutputCurrent();
    bottomCurrent = bottomMotor.getOutputCurrent();

    topAppliedVoltage = topMotor.getAppliedOutput() * topMotor.getBusVoltage();
    bottomAppliedVoltage = bottomMotor.getAppliedOutput() * bottomMotor.getBusVoltage();
  }

  @Override
  public void simulationPeriodic() {
    topVelocity = topSetpoint;
    bottomVelocity = bottomSetpoint;
  }

  private void setVelocity(
      Measure<Velocity<Angle>> topSpeed, Measure<Velocity<Angle>> bottomSpeed) {
    topSetpoint = topSpeed.in(RPM);
    bottomSetpoint = bottomSpeed.in(RPM);

    topController.setReference(topSpeed.in(RPM), ControlType.kVelocity);
    bottomController.setReference(bottomSpeed.in(RPM), ControlType.kVelocity);
  }

  private void setNoteStatus(boolean hasNote) {
    this.hasNote = hasNote;
  }

  public boolean hasNote() {
    return hasNote;
  }

  public Command intakeCommand() {
    return Commands.sequence(
    Commands.startEnd(
            () ->
                this.setVelocity(
                    IndexerConstants.TOP_INTAKING_SPEED, IndexerConstants.BOTTOM_INTAKING_SPEED),
            () -> this.setVelocity(RPM.of(0), RPM.of(0)),
            this)
        .onlyWhile(() -> !topNoteDetector.get()),
        Commands.runOnce(() -> setNoteStatus(true)));
  }

  public Command alignForShot() {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                this.setVelocity(
                    IndexerConstants.TOP_ALIGNING_SPEED, IndexerConstants.BOTTOM_ALIGNING_SPEED),
            this),
        Commands.waitUntil(() -> bottomNoteDetector.get()),
        Commands.runOnce(
            () ->
                this.setVelocity(
                    IndexerConstants.TOP_ALIGNING_SPEED,
                    IndexerConstants.BOTTOM_ALIGNING_SPEED.times(-1)),
            this),
        Commands.waitUntil(() -> !topNoteDetector.get()),
        Commands.runOnce(() -> this.setVelocity(RPM.of(0), RPM.of(0)), this));
  }

  public Command shootSpeaker() {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                this.setVelocity(
                    IndexerConstants.TOP_SHOOTING_SPEED, IndexerConstants.BOTTOM_SHOOTING_SPEED),
            this),
        Commands.waitSeconds(0.1),
        Commands.waitUntil(() -> !this.topNoteDetector.get()),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> this.setVelocity(RPM.of(0), RPM.of(0)), this));
  }

  public Command scoreAmp() {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                this.setVelocity(IndexerConstants.TOP_AMP_SPEED, IndexerConstants.BOTTOM_AMP_SPEED),
            this),
        Commands.waitUntil(() -> bottomNoteDetector.get()),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> this.setVelocity(RPM.of(0), RPM.of(0)), this));
  }
}
