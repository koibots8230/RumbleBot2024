package frc.lib.util;

public class FeedforwardConstants {
  public final double ks;
  public final double kv;
  public final double ka;
  public final double kg;

  public FeedforwardConstants(double ks, double kv) {
    this.ks = ks;
    this.kv = kv;
    this.ka = 0;
    this.kg = 0;
  }

  public FeedforwardConstants(double ks, double kv, double ka) {
    this.ks = ks;
    this.kv = kv;
    this.ka = ka;
    this.kg = 0;
  }

  public FeedforwardConstants(double ks, double kv, double ka, double kg) {
    this.ks = ks;
    this.kv = kv;
    this.ka = ka;
    this.kg = kg;
  }
}
