package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {

  private final CANSparkMax intakeMotor;

  private final SparkPIDController controller;

  public Intake() {

    intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);

    controller = intakeMotor.getPIDController();

    controller.setP(IntakeConstants.PID_GAINS.kp);
  }

  // private void setVelocity(Measure<Velocity<Angle>> setpoint) {
  //   controller.setReference(setpoint.in(RPM), ControlType.kVelocity);
  // }
  public Command setVelocity(Measure<Velocity<Angle>> setpoint) {
    // implicitly require `this`
    return this.runOnce(() -> controller.setReference(setpoint.in(RPM), ControlType.kVelocity));
  }
}
