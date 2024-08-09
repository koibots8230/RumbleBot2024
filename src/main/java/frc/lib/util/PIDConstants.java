package frc.lib.util;

public class PIDConstants {
  public final double kp;
  public final double ki;
  public final double kd;

  public PIDConstants(double kp, double ki, double kd) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
  }
}
