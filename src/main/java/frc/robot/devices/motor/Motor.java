package frc.robot.devices.motor;

import edu.wpi.first.units.*;

public interface Motor {
    void setP(double P);
    void setI(double I);
    void setD(double D);
    void setFF(double FF);
    void setVelocityFactor(double velocityFactor);
    void setPositionFactor(double positionFactor);
    void setHasAbsoluteEncoder(boolean hasAbsoluteEncoder);
    void setCANTimeout(Measure<Time> canTimeout);
    void setInverted(boolean inverted);
    void setCurrentLimit(Measure<Current> currentLimit);
    void setIdleMode(IdleMode idleMode);

    double getP();
    double getI();
    double getD();
    double getFF();
    double getVelocityFactor();
    double getPositionFactor();
    boolean getHasAbsoluteEncoder();
    boolean getInverted();
    IdleMode getIdleMode();

    void setVelocityAngle(Measure<Velocity<Angle>> velocity);
    void setVelocityDistance(Measure<Velocity<Distance>> velocity);
    void setPosition(Measure<Angle> position);
    Measure<Velocity<Angle>> getVelocityAngle();
    Measure<Velocity<Distance>> getVelocityDistance();
    Measure<Angle> getPosition();

    enum IdleMode {
        COAST,
        BREAK
    }

    record MotorDefinition(
            int ID,
            double P,
            double I,
            double D,
            double FF,
            double VelocityFactor,
            double PositionFactor,
            boolean HasAbsoluteEncoder,
            boolean inverted,
            Measure<Current> currentLimit,
            Motor.IdleMode idleMode) {
    }
}
