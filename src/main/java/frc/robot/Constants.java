package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.Drivetrain.PIDConstantsIO;
import frc.robot.subsystems.Drivetrain.FeedforwardConstantsIO;
import frc.robot.subsystems.Drivetrain.Wheel;
import frc.robot.subsystems.Drivetrain.MotorConstantsIO;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.*;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.MotorConfig;
import frc.lib.util.PIDGains;

public class Constants {
  public static class RobotConstants {
    public static final Measure<Distance> WIDTH = Inches.of(26.0);
    public static final Measure<Distance> LENGTH = Inches.of(26.0);
    public static final Measure<Distance> WHEEL_OFFSET = Inches.of(1.754419);
    public static final double DRIVE_POSITION_FACTOR = 1.0;
    public static final int GYRO_ID = 24;

    public static final Measure<Velocity<Distance>> MAX_LINEAR_SPEED =
            MetersPerSecond.of(4.125);
    public static final Measure<Velocity<Angle>> MAX_ANGULAR_VELOCITY =
            RadiansPerSecond.of(2 * Math.PI);
    public static final Measure<Velocity<Velocity<Distance>>> MAX_LINEAR_ACCELERATION =
            MetersPerSecondPerSecond.of(4);
    public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCELERATION =
            RadiansPerSecond.of(4 * Math.PI).per(Second);

    private static final int DRIVING_PINION_TEETH = 13;
    public static final double DRIVE_GEAR_RATIO = (45.0 * 22) / (DRIVING_PINION_TEETH * 15);
    public static final double TURN_GEAR_RATIO = (62.0 / 14) * 12;

    public static final Wheel DRIVE_WHEELS = new Wheel(Inches.of(1.5));

    public static final Measure<Voltage> NOMINAL_VOLTAGE = Volts.of(12);
  }

  public static class DriveConstants {

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

  public static class DrivetrainConstants {
    public static final double DRIVE_TURN_KS = 0.0;
    public static final PIDConstantsIO TURN_PID_CONSTANTS =
            new PIDConstantsIO(2.078, 0, 0, 35, 0, 0);
    public static final PIDConstantsIO DRIVE_PID_CONSTANTS =
            new PIDConstantsIO(5.5208e-10, 0, 0, 40, 0, 0);
    public static final FeedforwardConstantsIO DRIVE_FEEDFORWARD_CONSTANTS =
            new FeedforwardConstantsIO(0.11386, 2.6819, 0.16507, 0, 2.65, 0);

    public static final double DEADBAND = 0.025;

    public static final PIDConstantsIO ANGLE_ALIGNMENT_PID_CONSTANTS =
            new PIDConstantsIO(0.925, 0, 0, 3.5, 0, 0);

    public static final SwerveDriveKinematics SWERVE_KINEMATICS =
            new SwerveDriveKinematics(
                    new Translation2d(
                            RobotConstants.LENGTH.divide(2),
                            RobotConstants.WIDTH.divide(2)), // Front Left
                    new Translation2d(
                            RobotConstants.LENGTH.divide(2),
                            RobotConstants.WIDTH.divide(-2)), // Front Right
                    new Translation2d(
                            RobotConstants.LENGTH.divide(-2),
                            RobotConstants.WIDTH.divide(2)), // Back Left
                    new Translation2d(
                            RobotConstants.LENGTH.divide(-2),
                            RobotConstants.WIDTH.divide(-2)) // Back Right
            );
    public static final PIDConstantsIO VX_CONTROLLER = new PIDConstantsIO(1.5, 0, 0, 0, 0, 0);
    public static final PIDConstantsIO VY_CONTROLLER = new PIDConstantsIO(0, 0, 0, 0, 0, 0);
    public static final PIDConstantsIO VTHETA_CONTROLLER = new PIDConstantsIO(0, 0, 0, 0, 0, 0);

    public static final MotorConstantsIO DRIVE =
            new MotorConstantsIO(false, 60, IdleMode.kBrake);
    public static final MotorConstantsIO TURN =
            new MotorConstantsIO(false, 30, IdleMode.kBrake);

    public static final Measure<Time> CAN_TIMEOUT =
            Milliseconds.of(
                    20); // Default value, but if CAN utilization gets too high, pop it to 0, or
    // bump it up+

    public static final int ENCODER_SAMPLES_PER_AVERAGE = 100;

    public static final Measure<Angle> TURNING_ENCODER_POSITION_FACTOR =
            Radians.of(2 * Math.PI);
    public static final Measure<Velocity<Angle>> TURNING_ENCODER_VELOCITY_FACTOR =
            RadiansPerSecond.of((2 * Math.PI) / 60.0);

    public static final Measure<Distance> DRIVING_ENCODER_POSITION_FACTOR =
            Inches.of((1.5 * 2 * Math.PI) / RobotConstants.DRIVE_GEAR_RATIO);
    public static final Measure<Velocity<Distance>> DRIVING_ENCODER_VELOCITY_FACTOR =
            MetersPerSecond.of(
                    ((RobotConstants.DRIVE_WHEELS.radius.in(Meters) * 2 * Math.PI)
                            / RobotConstants.DRIVE_GEAR_RATIO)
                            / 60.0);

    public static final int DRIVE_ENCODER_SAMPLING_DEPTH = 2;

    public static final double SHOOTER_ALLOWED_ERROR = 0.1;

    public static class DeviceIDs { //TODO: ACTUALLY SET CANIDS
      public static final int FRONT_LEFT_DRIVE = 36;
      public static final int FRONT_LEFT_TURN = 31;
      public static final int FRONT_RIGHT_DRIVE = 37;
      public static final int FRONT_RIGHT_TURN = 38;
      public static final int BACK_LEFT_DRIVE = 34;
      public static final int BACK_LEFT_TURN = 35;
      public static final int BACK_RIGHT_DRIVE = 32;
      public static final int BACK_RIGHT_TURN = 33;
    }
  }

  public static class IntakeConstants {

    public static final Measure<Velocity<Angle>> INTAKE_MOTOR_SETPOINT = RPM.of(0);

    public static final int INTAKE_MOTOR_PORT = 5;

    public static final double INTAKE_FEED_FORWARD = 0.0;

    public static final PIDGains PID_GAINS =
        new PIDGains.Builder().kp(0.0).ki(0.0).kf(0.0).kd(0.0).build();
  }
}
