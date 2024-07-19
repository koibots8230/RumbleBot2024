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

    public Motor create(int ID, boolean isReal) {
        if (isReal) {
            return new MotorCANSparkMax(ID);
        } else {
            MotorSim motorSim = new MotorSim(ID);
            motorSims.add(motorSim);
            return motorSim;
        }
    }

    public void updateSims() {
        for (MotorSim motorSim : motorSims) motorSim.update();
    }
}
