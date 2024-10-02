package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.*;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.MotorConfig;
import frc.lib.util.PIDGains;
import frc.lib.util.Wheel;

public class Constants {
  public static class RobotConstants {
    public static final Measure<Distance> WIDTH = Inches.of(26.0);
    public static final Measure<Distance> LENGTH = Inches.of(26.0);

    public static final Measure<Voltage> NOMINAL_VOLTAGE = Volts.of(12);

    public static final Measure<Time> CAN_TIMEOUT = Milliseconds.of(20);

    public static final double JOYSTICK_DEADBAND = 0.05;
  }

  public static class FieldConstants {

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
  public static class ShooterPivotConstants {

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

    public static final double TOP_MOTOR_SETPOINT_SPEAKER = 600; // todo set actaul val for setpoint
    public static final double BOTTOM_MOTOR_SETPOINT_SPEAKER =
        1000; // todo set actaul val for setpoint

    // 2

    public static final double REST_SETPOINT = 500;

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

  public static class SwerveConstants {

    public static final Measure<Velocity<Distance>> MAX_LINEAR_SPEED = MetersPerSecond.of(4.125);

    public static final Measure<Velocity<Angle>> MAX_ANGULAR_VELOCITY =
        RadiansPerSecond.of(2 * Math.PI);

    public static final Measure<Velocity<Angle>> MAX_TURN_VELOCITY =
        RadiansPerSecond.of(2 * Math.PI);
    public static final Measure<Velocity<Velocity<Angle>>> MAX_TURN_ACCELERATION =
        RadiansPerSecond.per(Second).of(Math.PI * 4);

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(RobotConstants.LENGTH.divide(2), RobotConstants.WIDTH.divide(2)),
            new Translation2d(RobotConstants.LENGTH.divide(2), RobotConstants.WIDTH.divide(-2)),
            new Translation2d(RobotConstants.LENGTH.divide(-2), RobotConstants.WIDTH.divide(2)),
            new Translation2d(RobotConstants.LENGTH.divide(-2), RobotConstants.WIDTH.divide(-2)));

    public static final PIDGains DRIVE_PID_GAINS = new PIDGains.Builder().kp(0).build();
    public static final FeedforwardGains DRIVE_FF_GAINS =
        new FeedforwardGains.Builder().kv(0).build();

    public static final PIDGains TURN_PID_GAINS = new PIDGains.Builder().kp(0).build();
    public static final FeedforwardGains TURN_FF_GAINS =
        new FeedforwardGains.Builder().ks(0).kv(0).build();

    public static final MotorConfig DRIVE_MOTOR_CONFIG =
        new MotorConfig.Builder().currentLimit(60).build();
    public static final MotorConfig TURN_MOTOR_CONFIG =
        new MotorConfig.Builder().currentLimit(30).build();

    public static final Rotation2d[] ANGLE_OFFSETS =
        new Rotation2d[] {
          Rotation2d.fromRadians((3 * Math.PI) / 2),
          Rotation2d.fromRadians(Math.PI),
          Rotation2d.fromRadians(0),
          Rotation2d.fromRadians(Math.PI / 2)
        };

    private static final int DRIVING_PINION_TEETH = 13;
    public static final double DRIVE_GEAR_RATIO = (45.0 * 22) / (DRIVING_PINION_TEETH * 15);

    public static final Wheel WHEELS = new Wheel(Inches.of(1.5));

    public static final Measure<Angle> TURN_ENCODER_POSITION_FACTOR = Radians.of(2 * Math.PI);
    public static final Measure<Velocity<Angle>> TURN_ENCODER_VELOCITY_FACTOR =
        RadiansPerSecond.of((2 * Math.PI) / 60.0);

    public static final Measure<Distance> DRIVE_ENCODER_POSITION_FACTOR =
        Inches.of((1.5 * 2 * Math.PI) / DRIVE_GEAR_RATIO);
    public static final Measure<Velocity<Distance>> DRIVE_ENCODER_VELOCITY_FACTOR =
        MetersPerSecond.of(((WHEELS.radius.in(Meters) * 2 * Math.PI) / DRIVE_GEAR_RATIO) / 60.0);

    public static final int FRONT_LEFT_DRIVE_ID = 36;
    public static final int FRONT_LEFT_TURN_ID = 31;
    public static final int FRONT_RIGHT_DRIVE_ID = 37;
    public static final int FRONT_RIGHT_TURN_ID = 38;
    public static final int BACK_LEFT_DRIVE_ID = 34;
    public static final int BACK_LEFT_TURN_ID = 35;
    public static final int BACK_RIGHT_DRIVE_ID = 32;
    public static final int BACK_RIGHT_TURN_ID = 33;

    public static final int GYRO_ID = 24;
  }

  public static class IntakeConstants {

    public static final Measure<Velocity<Angle>> INTAKE_MOTOR_SETPOINT = RPM.of(0);

    public static final int INTAKE_MOTOR_PORT = 5;

    public static final double INTAKE_FEED_FORWARD = 0.0;

    public static final PIDGains PID_GAINS =
        new PIDGains.Builder().kp(0.0).ki(0.0).kf(0.0).kd(0.0).build();
  }

  public static class VisionConstants {
    public static final int ACTIVE_CAMERAS = 3;

    public static final Pose2d[] CAMERA_POSITIONS = {
      new Pose2d(-0.1524, -0.26035, new Rotation2d(Math.toRadians(90))),
      new Pose2d(-0.1651, 0.0254, new Rotation2d(Math.toRadians(180))),
      new Pose2d(-0.1524, 0.26035, new Rotation2d(Math.toRadians(270)))
    }; // x is forward, y is left, counterclockwise on rotation

    public static final String[][] TOPIC_NAMES = {
      {"Cam1Tvec", "Cam1Rvec", "Cam1Ids"},
      {"Cam2Tvec", "Cam2Rvec", "Cam2Ids"},
      {"Cam3Tvec", "Cam3Rvec", "Cam3Ids"}
    };

    public static final double[] VECTOR_DEFAULT_VALUE = {0};
    public static final int ID_DEFAULT_VALUE = 0;

    public static final Measure<Distance> MAX_MEASUREMENT_DIFFERENCE = Meters.of(1.5);
    public static final Rotation2d MAX_ANGLE_DIFFERENCE = Rotation2d.fromDegrees(10);

    public static final double ROTATION_STDEV = 50 * Math.PI;
    public static final double TRANSLATION_STDEV_ORDER = 2;
    public static final double TRANSLATION_STDEV_SCALAR = 2;
  }
}
