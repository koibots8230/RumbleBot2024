package frc.robot.devices.motor;

import com.revrobotics.*;
import edu.wpi.first.units.*;

public class MotorCANSparkMax implements Motor {
  private boolean useAbsoluteEncoder;
  private final CANSparkMax canSparkMax;
  private final SparkPIDController controller;
  private final RelativeEncoder relativeEncoder;
  private AbsoluteEncoder absoluteEncoder;

  public MotorCANSparkMax(MotorDefinition motorDefinition) {
    useAbsoluteEncoder = false;
    canSparkMax = new CANSparkMax(motorDefinition.ID(), CANSparkLowLevel.MotorType.kBrushless);
    relativeEncoder = canSparkMax.getEncoder();
    controller = canSparkMax.getPIDController();
    controller.setFeedbackDevice(relativeEncoder);
    setP(motorDefinition.P());
    setI(motorDefinition.I());
    setD(motorDefinition.D());
    setFF(motorDefinition.FF());
    setVelocityFactor(motorDefinition.VelocityFactor());
    setPositionFactor(motorDefinition.PositionFactor());
    setHasAbsoluteEncoder(motorDefinition.HasAbsoluteEncoder());
    setInverted(motorDefinition.inverted());
    setCurrentLimit(motorDefinition.currentLimit());
    setIdleMode(motorDefinition.idleMode());
  }

  @Override
  public void setP(double P) {
    controller.setP(P);
  }

  @Override
  public void setI(double I) {
    controller.setI(I);
  }

  @Override
  public void setD(double D) {
    controller.setD(D);
  }

  @Override
  public void setFF(double FF) {
    controller.setFF(FF);
  }

  @Override
  public void setVelocityFactor(double velocityFactor) {
    relativeEncoder.setVelocityConversionFactor(velocityFactor);
    if (useAbsoluteEncoder) absoluteEncoder.setVelocityConversionFactor(velocityFactor);
  }

  @Override
  public void setPositionFactor(double positionFactor) {
    relativeEncoder.setPositionConversionFactor(positionFactor);
    if (useAbsoluteEncoder) absoluteEncoder.setPositionConversionFactor(positionFactor);
  }

  @Override
  public void setHasAbsoluteEncoder(boolean hasAbsoluteEncoder) {
    useAbsoluteEncoder = hasAbsoluteEncoder;

    if (useAbsoluteEncoder) {
      absoluteEncoder = canSparkMax.getAbsoluteEncoder();
      absoluteEncoder.setVelocityConversionFactor(relativeEncoder.getVelocityConversionFactor());
      absoluteEncoder.setPositionConversionFactor(relativeEncoder.getPositionConversionFactor());
    } else {
      absoluteEncoder = null;
    }
  }

  @Override
  public void setCANTimeout(Measure<Time> canTimeout) {
    canSparkMax.setCANTimeout((int) canTimeout.in(Units.Milliseconds));
  }

  @Override
  public void setInverted(boolean inverted) {
    canSparkMax.setInverted(inverted);
  }

  @Override
  public void setCurrentLimit(Measure<Current> currentLimit) {
    canSparkMax.setSmartCurrentLimit((int) currentLimit.in(Units.Amps));
  }

  @Override
  public void setIdleMode(IdleMode idleMode) {
    canSparkMax.setIdleMode(
        idleMode == IdleMode.BREAK ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
  }

  @Override
  public double getP() {
    return controller.getP();
  }

  @Override
  public double getI() {
    return controller.getI();
  }

  @Override
  public double getD() {
    return controller.getD();
  }

  @Override
  public double getFF() {
    return controller.getFF();
  }

  @Override
  public double getVelocityFactor() {
    return relativeEncoder.getVelocityConversionFactor();
  }

  @Override
  public double getPositionFactor() {
    return relativeEncoder.getPositionConversionFactor();
  }

  @Override
  public boolean getHasAbsoluteEncoder() {
    return useAbsoluteEncoder;
  }

  @Override
  public boolean getInverted() {
    return canSparkMax.getInverted();
  }

  @Override
  public IdleMode getIdleMode() {
    return canSparkMax.getIdleMode() == CANSparkBase.IdleMode.kBrake
        ? IdleMode.BREAK
        : IdleMode.COAST;
  }

  @Override
  public void setVelocityAngle(Measure<Velocity<Angle>> velocity) {
    controller.setReference(velocity.in(Units.RPM), CANSparkBase.ControlType.kVelocity);
  }

  @Override
  public void setVelocityDistance(Measure<Velocity<Distance>> velocity) {
    controller.setReference(velocity.in(Units.MetersPerSecond), CANSparkBase.ControlType.kVelocity);
  }

  @Override
  public void setPosition(Measure<Angle> position) {
    controller.setReference(position.in(Units.Rotations), CANSparkBase.ControlType.kPosition);
  }

  @Override
  public Measure<Velocity<Angle>> getVelocityAngle() {
    return Units.RPM.of(relativeEncoder.getVelocity());
  }

  @Override
  public Measure<Velocity<Distance>> getVelocityDistance() {
    return Units.MetersPerSecond.of(relativeEncoder.getVelocity());
  }

  @Override
  public Measure<Angle> getPosition() {
    return Units.Rotations.of(relativeEncoder.getPosition());
  }
}
