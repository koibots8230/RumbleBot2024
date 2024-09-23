package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import frc.lib.motor.Motor;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.MotorConfig;
import frc.lib.util.PIDGains;

public class Constants {
  public static class RobotConstants {
    public static final Measure<Distance> WIDTH = Inches.of(26.0);
    public static final Measure<Distance> LENGTH = Inches.of(26.0);
    public static final Measure<Distance> WHEEL_OFFSET = Inches.of(1.754419);
    public static final double DRIVE_POSITION_FACTOR = 1.0;
  }

  public  static class FiledConstants {

    public static final Pose2d RED_SPEAKER_POSE = new Pose2d(16.58, 5.55, new Rotation2d(Math.PI));

    public static final Pose2d BLUE_SPEAKER_POSE = new Pose2d(0, 5.55, new Rotation2d());
  }

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

    public static final double AUTO_ANGLE_SLOPE = 0.0;
    public static final double Y_INTERCEPT = 0.0;

    public static final Rotation2d REST_POSITION = Rotation2d.fromDegrees(21);
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

  public static class DrivetrainConstants {
    private static final double DRIVE_P = 1.0;
    private static final double DRIVE_I = 0.0;
    private static final double DRIVE_D = 0.0;
    private static final double DRIVE_FF = 1.0; // Feedforward
    private static final double DRIVE_VF = 1.0; // Velocity Factor
    private static final double DRIVE_PF = 1.0; // Position Factor
    private static final Measure<Current> DRIVE_CL = Amps.of(40.0); // Current Limit
    private static final Motor.IdleMode DRIVE_IM = Motor.IdleMode.BREAK;

    private static final double TURN_P = 1.0;
    private static final double TURN_I = 0.0;
    private static final double TURN_D = 0.0;
    private static final double TURN_FF = 1.0; // Feedforward
    private static final double TURN_VF = 1.0; // Velocity Factor
    private static final double TURN_PF = 1.0; // Position Factor
    private static final Measure<Current> TURN_CL = Amps.of(30.0); // Current Limit
    private static final Motor.IdleMode TURN_IM = Motor.IdleMode.BREAK;

    public static final Motor.MotorDefinition FRONT_LEFT_DRIVE =
        new Motor.MotorDefinition(
            30, DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_FF, DRIVE_VF, DRIVE_PF, false, false, DRIVE_CL,
            DRIVE_IM);
    public static final Motor.MotorDefinition FRONT_LEFT_TURN =
        new Motor.MotorDefinition(
            30, TURN_P, TURN_I, TURN_D, TURN_FF, TURN_VF, TURN_PF, false, false, TURN_CL, TURN_IM);
    public static final Motor.MotorDefinition FRONT_RIGHT_DRIVE =
        new Motor.MotorDefinition(
            30, DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_FF, DRIVE_VF, DRIVE_PF, false, false, DRIVE_CL,
            DRIVE_IM);
    public static final Motor.MotorDefinition FRONT_RIGHT_TURN =
        new Motor.MotorDefinition(
            30, TURN_P, TURN_I, TURN_D, TURN_FF, TURN_VF, TURN_PF, false, false, TURN_CL, TURN_IM);
    public static final Motor.MotorDefinition BACK_LEFT_DRIVE =
        new Motor.MotorDefinition(
            30, DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_FF, DRIVE_VF, DRIVE_PF, false, false, DRIVE_CL,
            DRIVE_IM);
    public static final Motor.MotorDefinition BACK_LEFT_TURN =
        new Motor.MotorDefinition(
            30, TURN_P, TURN_I, TURN_D, TURN_FF, TURN_VF, TURN_PF, false, false, TURN_CL, TURN_IM);
    public static final Motor.MotorDefinition BACK_RIGHT_DRIVE =
        new Motor.MotorDefinition(
            30, DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_FF, DRIVE_VF, DRIVE_PF, false, false, DRIVE_CL,
            DRIVE_IM);
    public static final Motor.MotorDefinition BACK_RIGHT_TURN =
        new Motor.MotorDefinition(
            30, TURN_P, TURN_I, TURN_D, TURN_FF, TURN_VF, TURN_PF, false, false, TURN_CL, TURN_IM);
  }

  public static class IntakeConstants {

    public static final Measure<Velocity<Angle>> INTAKE_MOTOR_SETPOINT = RPM.of(0);

    public static final int INTAKE_MOTOR_PORT = 5;

    public static final double INTAKE_FEED_FORWARD = 0.0;

    public static final PIDGains PID_GAINS =
        new PIDGains.Builder().kp(0.0).ki(0.0).kf(0.0).kd(0.0).build();
  }
}
