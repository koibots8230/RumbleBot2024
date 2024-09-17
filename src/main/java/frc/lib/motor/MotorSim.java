package frc.lib.motor;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;

public class MotorSim implements Motor {
  private final SimDevice simDevice;
  private final SimDouble simVelocity;
  private final SimDouble simVelocityFactor;
  private final SimDouble simVelocityPreconv;
  private final SimDouble simPosition;
  private final SimDouble simPositionFactor;
  private final SimDouble simPositionPreconv;
  private boolean usePosition;
  private Measure<Velocity<Angle>> velocity;
  private Measure<Angle> position;
  private Measure<Angle> prevPosition;
  private double velocityFactor;
  private double positionFactor;
  private final PIDController pidController;
  private double FF;
  private boolean useAbsoluteEncoder;
  private boolean inverted;
  private IdleMode idleMode;

  public MotorSim(MotorDefinition motorDefinition) {
    velocity = Units.RPM.zero();
    position = Units.Rotations.zero();
    prevPosition = Units.Rotations.zero();
    simDevice =
        SimDevice.create("Motor [".concat(String.valueOf(motorDefinition.ID())).concat("]"));
    simVelocity = simDevice.createDouble("Velocity (Likely RPM)", SimDevice.Direction.kOutput, 0);
    simVelocityFactor = simDevice.createDouble("Velocity Factor", SimDevice.Direction.kOutput, 0);
    simVelocityPreconv =
        simDevice.createDouble("Velocity Pre-conversion (RPM)", SimDevice.Direction.kOutput, 0);
    simPosition =
        simDevice.createDouble("Position (Likely Rotations)", SimDevice.Direction.kOutput, 0);
    simPositionFactor = simDevice.createDouble("Position Factor", SimDevice.Direction.kOutput, 0);
    simPositionPreconv =
        simDevice.createDouble(
            "Position Pre-conversion (Rotations)", SimDevice.Direction.kOutput, 0);
    pidController = new PIDController(0, 0, 0);
    velocityFactor = 1;
    positionFactor = 1;
    FF = 0;
    inverted = false;
    idleMode = IdleMode.COAST;

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
    pidController.setP(P);
  }

  @Override
  public void setI(double I) {
    pidController.setI(I);
  }

  @Override
  public void setD(double D) {
    pidController.setD(D);
  }

  @Override
  public void setFF(double FF) {
    this.FF = FF; // TODO: implement a simple feedforward
  }

  @Override
  public void setVelocityFactor(double velocityFactor) {
    this.velocityFactor = velocityFactor;
  }

  @Override
  public void setPositionFactor(double positionFactor) {
    this.positionFactor = positionFactor;
  }

  @Override
  public void setHasAbsoluteEncoder(boolean hasAbsoluteEncoder) {
    useAbsoluteEncoder = hasAbsoluteEncoder;
  }

  @Override
  public void setCANTimeout(Measure<Time> canTimeout) {}

  @Override
  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }

  @Override
  public void setCurrentLimit(Measure<Current> currentLimit) {}

  @Override
  public void setIdleMode(IdleMode idleMode) {
    this.idleMode = idleMode;
  }

  @Override
  public double getP() {
    return pidController.getP();
  }

  @Override
  public double getI() {
    return pidController.getI();
  }

  @Override
  public double getD() {
    return pidController.getD();
  }

  @Override
  public double getFF() {
    return FF;
  }

  @Override
  public double getVelocityFactor() {
    return velocityFactor;
  }

  @Override
  public double getPositionFactor() {
    return positionFactor;
  }

  @Override
  public boolean getHasAbsoluteEncoder() {
    return useAbsoluteEncoder;
  }

  @Override
  public boolean getInverted() {
    return inverted;
  }

  @Override
  public IdleMode getIdleMode() {
    return idleMode;
  }

  @Override
  public void setVelocityAngle(Measure<Velocity<Angle>> velocity) {
    usePosition = false;
    this.velocity = velocity;
  }

  @Override
  public void setVelocityDistance(Measure<Velocity<Distance>> velocity) {
    usePosition = false;
    this.velocity = Units.RPM.of(velocity.in(Units.MetersPerSecond));
  }

  @Override
  public void setPosition(Rotation2d position) {
    usePosition = true;
    this.position = Units.Rotations.of(position.getRotations());
  }

  @Override
  public Measure<Velocity<Angle>> getVelocityAngle() {
    return velocity;
  }

  @Override
  public Measure<Velocity<Distance>> getVelocityDistance() {
    return Units.MetersPerSecond.of(velocity.in(Units.RPM));
  }

  @Override
  public Rotation2d getPosition() {
    return Rotation2d.fromRotations(position.in(Units.Rotations));
  }

  public void update() {
    simVelocityFactor.set(velocityFactor);
    simPositionFactor.set(positionFactor);

    if (!usePosition) {
      simVelocity.set(velocity.in(Units.RPM));
      simVelocityPreconv.set(velocity.divide(velocityFactor).in(Units.RPM));

      position =
          position.plus(Units.Rotations.of(velocity.in(Units.Rotations.per(Units.Minute)) / 50));

      simPosition.set(position.in(Units.Rotations));
      simPositionPreconv.set(position.divide(positionFactor).in(Units.Rotations));
    } else {
      simPosition.set(position.in(Units.Rotations));
      simPositionPreconv.set(position.divide(positionFactor).in(Units.Rotations));

      velocity = position.minus(prevPosition).per(Units.Minute.one().divide(50));

      simVelocity.set(velocity.in(Units.RPM));
      simVelocityPreconv.set(velocity.divide(velocityFactor).in(Units.RPM));
    }

    prevPosition = position;
  }
}
