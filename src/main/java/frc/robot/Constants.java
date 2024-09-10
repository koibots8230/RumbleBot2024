package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.units.*;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.lib.util.FeedforwardConstants;
import frc.lib.util.FeedforwardConstantsIO;
import frc.lib.util.MotorConstantsIO;
import frc.lib.util.PIDConstants;
import frc.lib.util.PIDConstantsIO;

public class Constants {

  public static class ElevatorConstants {

    public static final PIDConstants REAL_FEEDBACK_CONSTANTS = new PIDConstants(0, 0, 0);
    public static final PIDConstants SIM_FEEDBACK_CONSTANTS = new PIDConstants(0.7, 0, 0);

    public static final FeedforwardConstants REAL_FEEDFORWARD_CONSTANTS =
        new FeedforwardConstants(0, 0, 0, 0);
    public static final FeedforwardConstants SIM_FEEDFORWARD_CONSTANTS =
        new FeedforwardConstants(0, 9, 0, 0.143607);

    public static final Measure<Velocity<Distance>> MAX_VELOCITY = MetersPerSecond.of(1);
    public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCELERATION =
        MetersPerSecondPerSecond.of(1);

    public static final double GEAR_RATIO = 10;
    public static final Measure<Distance> DRUM_SIZE = Inches.of(1);

    public static final Measure<Distance> MAX_HEIGHT = Inches.of(6);

    public static final int LEFT_MOTOR_PORT = 1;
    public static final int RIGHT_MOTOR_PORT = 2;
  }

  public static class ShooterConstants {
    // 1

    public static final Measure<Velocity<Angle>> TOP_MOTOR_SETPOINT_APM =
        RPM.of(600); // todo set actaul val for setpoint
    public static final Measure<Velocity<Angle>> TOP_MOTOR_SETPOINT_SPEAKER =
        RPM.of(600); // todo set actaul val for setpoint

    public static final Measure<Velocity<Angle>> BOTTOM_MOTOR_SEETPOINT_APM =
        RPM.of(1000); // todo set actaul val for setpoint
    public static final Measure<Velocity<Angle>> BOTTOM_MOTOR_SETPOINT_SPEAKER =
        RPM.of(1000); // todo set actaul val for setpoint

    // 2

    public static final PIDConstantsIO TOP_PID_CONSTANTS_IO =
        new PIDConstantsIO(0, 0, 0, .008, 0, 0);

    public static final FeedforwardConstantsIO TOP_FEEDFORWARD_CONSTANTS_IO =
        new FeedforwardConstantsIO(0, 0, 0, 0.0021); // only care about Kv set later

    public static final PIDConstantsIO BOTTOM_PID_CONSTANTS_IO =
        new PIDConstantsIO(0, 0, 0, .008, 0, 0);

    public static final FeedforwardConstantsIO BOTTOM_FEEDFORWARD_CONSTANTS_IO =
        new FeedforwardConstantsIO(0, 0, 0, 0.0021); // only care about Kv set later

    // 3

    public static final int SHOOTER_RANGE = 20;

    // 4

    public static final MotorConstantsIO TOP_MOTOR_CONSTANTS_IO =
        new MotorConstantsIO(true, 60, IdleMode.kBrake);

    public static final MotorConstantsIO BOTTOM_MOTOR_CONSTANTS_IO =
        new MotorConstantsIO(false, 60, IdleMode.kBrake);

    // 6

    public static final int TOP_SHOOTER_PORT = 1;

    public static final int BOTTOM_SHOOTER_PORT = 2;
  }
}
