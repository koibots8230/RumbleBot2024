package frc.robot.devices.motor;

import java.util.ArrayList;
import java.util.List;

public class MotorFactory {
  private final List<MotorSim> motorSims;

  private static final MotorFactory INSTANCE = new MotorFactory();

  public static MotorFactory get() {
    return INSTANCE;
  }

  private MotorFactory() {
    motorSims = new ArrayList<>();
  }

  public Motor create(Motor.MotorDefinition motorDefinition, boolean isReal) {
    if (isReal) {
      return new MotorCANSparkMax(motorDefinition);
    } else {
      MotorSim motorSim = new MotorSim(motorDefinition);
      motorSims.add(motorSim);
      return motorSim;
    }
  }

  public void updateSims() {
    for (MotorSim motorSim : motorSims) motorSim.update();
  }
}
