package frc.lib.util;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class Wheel {
  public final Measure<Distance> circumfrence;
  public final Measure<Distance> radius;

  public Wheel(Measure<Distance> radius) {
    this.radius = radius;
    circumfrence = radius.times(2 * Math.PI);
  }
}
