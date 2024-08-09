package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import frc.lib.util.FeedforwardConstants;
import frc.lib.util.PIDConstants;

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
}
