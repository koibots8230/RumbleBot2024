package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
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
    controller.setI(IntakeConstants.PID_GAINS.ki);
    controller.setD(IntakeConstants.PID_GAINS.kd);
    controller.setFF(IntakeConstants.INTAKE_FEED_FORWARD);
  }

  private void run(Measure<Velocity<Angle>> setpoint){
    controller.setReference(setpoint.in(RPM), ControlType.kVelocity);
  }
}
