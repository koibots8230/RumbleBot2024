package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.MotorConfig;
import frc.lib.util.PIDGains;

public class Constants {

  public static class IndexerConstants {
    public static final PIDGains TOP_FEEDBACK_GAINS = new PIDGains.Builder().kp(0).build();
    public static final PIDGains BOTTOM_FEEDBACK_GAINS = new PIDGains.Builder().kp(0).build();

    public static final FeedforwardGains TOP_FEEDFORWARD_GAINS =
        new FeedforwardGains.Builder().kv(0).build();
    public static final FeedforwardGains BOTTOM_FEEDFORWARD_GAINS =
        new FeedforwardGains.Builder().kv(0).build();

    public static final Measure<Velocity<Angle>> TOP_INTAKING_SPEED = RPM.of(200);
    public static final Measure<Velocity<Angle>> BOTTOM_INTAKING_SPEED = RPM.of(400);

    public static final Measure<Velocity<Angle>> TOP_AMP_SPEED = RPM.of(-200);
    public static final Measure<Velocity<Angle>> BOTTOM_AMP_SPEED = RPM.of(-400);

    public static final Measure<Velocity<Angle>> TOP_ALIGNING_SPEED = RPM.of(400);
    public static final Measure<Velocity<Angle>> BOTTOM_ALIGNING_SPEED = RPM.of(400);

    public static final Measure<Velocity<Angle>> TOP_SHOOTING_SPEED = RPM.of(600);
    public static final Measure<Velocity<Angle>> BOTTOM_SHOOTING_SPEED = RPM.of(1200);

    public static final double TOP_GEAR_RATIO = 1.0 / 27.0;
    public static final double BOTTOM_GEAR_RATIO = 1.0 / 27.0;

    public static final MotorConfig TOP_MOTOR_CONFIG =
        new MotorConfig.Builder().currentLimit(30).build();
    public static final MotorConfig BOTTOM_MOTOR_CONFIG =
        new MotorConfig.Builder().currentLimit(30).build();

    public static final int TOP_MOTOR_PORT = 5;
    public static final int BOTTOM_MOTOR_PORT = 6;

    public static final int TOP_NOTE_DETECTOR_PORT = 0;
    public static final int BOTTOM_NOTE_DETECTOR_PORT = 1;
  }

  public static class ElevatorConstants {

    public static final PIDGains PID_GAINS = new PIDGains.Builder().kp(0.0).build();
    public static final PIDGains SIM_PID_GAINS = new PIDGains.Builder().kp(0.7).build();

    public static final FeedforwardGains FEEDFORWARD_GAINS =
        new FeedforwardGains.Builder().ks(0.0).kv(0.0).ka(0.0).kg(0.0).build();
    public static final FeedforwardGains SIM_FEEDFORWARD_GAINS =
        new FeedforwardGains.Builder().ks(0.0).kv(9.0).ka(0.0).kg(0.143607).build();

    public static final Measure<Velocity<Distance>> MAX_VELOCITY = MetersPerSecond.of(1);
    public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCELERATION =
        MetersPerSecondPerSecond.of(1);

    public static final Measure<Distance> AMP_POSITION = Inches.of(6);
    public static final Measure<Distance> SHOOTING_POSITION = Inches.of(0.1);

    public static final double GEAR_RATIO = 10;
    public static final Measure<Distance> DRUM_SIZE = Inches.of(1);

    public static final Measure<Distance> MAX_HEIGHT = Inches.of(6);

    public static final Measure<Distance> ALLOWED_ERROR = Inches.of(0.05);

    public static final MotorConfig LEFT_MOTOR_CONFIG =
        new MotorConfig.Builder().currentLimit(60).build();
    public static final MotorConfig RIGHT_MOTOR_CONFIG =
        new MotorConfig.Builder().currentLimit(60).build();

    public static final int LEFT_MOTOR_PORT = 3;
    public static final int RIGHT_MOTOR_PORT = 4;
  }

  //  TODO add actual values into here
  public static class ShooterPivot {

    public static final int SHOOTER_PIVOT_MOTER = 0;

    public static final Rotation2d MAX_VELOCITY = Rotation2d.fromDegrees(30);
    public static final Rotation2d MAX_ACCLERATION = Rotation2d.fromDegrees(15);

    public static final FeedforwardGains FEEDFORWARD_GAINS =
        new FeedforwardGains.Builder().ka(0.0).kg(0.0).ks(0.0).kv(0.0).build();

    public static final PIDGains PID_GAINS =
        new PIDGains.Builder().kp(0.0).ki(0.0).kf(0.0).kd(0.0).build();

    public static final double a = 0.0;
    public static final double b = 0.0;

  }

  public static class ShooterConstants {
    // 1

    public static final Measure<Velocity<Angle>> TOP_MOTOR_SETPOINT_SPEAKER =
        RPM.of(600); // todo set actaul val for setpoint
    public static final Measure<Velocity<Angle>> BOTTOM_MOTOR_SETPOINT_SPEAKER =
        RPM.of(1000); // todo set actaul val for setpoint

    // 2

    public static final PIDGains PID_GAINS = new PIDGains.Builder().kp(0.008).build();
    public static final FeedforwardGains FEEDFORWARD_GAINS =
        new FeedforwardGains.Builder().kv(0.0021).build();

    // 3

    public static final int SHOOTER_RANGE = 20;

    // 4

    public static final MotorConfig TOP_MOTOR_CONFIG =
        new MotorConfig.Builder().inverted(true).currentLimit(60).idleMode(IdleMode.kBrake).build();

    public static final MotorConfig BOTTOM_MOTOR_CONFIG =
        new MotorConfig.Builder()
            .inverted(false)
            .currentLimit(60)
            .idleMode(IdleMode.kBrake)
            .build();

    // 6

    public static final int TOP_SHOOTER_PORT = 1;

    public static final int BOTTOM_SHOOTER_PORT = 2;
  }

  public static class IntakeConstants {

    public static final Measure<Velocity<Angle>> INTAKE_MOTOR_SETPOINT = RPM.of(0);

    public static final int INTAKE_MOTOR_PORT = 5;

    public static final double INTAKE_FEED_FORWARD = 0.0;

    public static final PIDGains PID_GAINS =
        new PIDGains.Builder().kp(0.0).ki(0.0).kf(0.0).kd(0.0).build();
  }
}
