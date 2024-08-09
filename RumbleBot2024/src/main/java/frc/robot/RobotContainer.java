// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import monologue.Logged;
import monologue.Monologue;

public class RobotContainer implements Logged {

  private final Shooter shooterSubsystem;

  private final boolean isReal;

  private final XboxController controller;

  public RobotContainer(boolean isReal) {

    this.isReal = isReal;

    controller = new XboxController(0);

    shooterSubsystem = new Shooter(isReal);

    Monologue.setupMonologue(
        this, "Robot", false,
        false);

    configureBindings();
  }

  private void configureBindings() {

    Trigger shooter = new Trigger(() -> controller.getRawButton(1));
    shooter
        .onTrue(new InstantCommand(() -> shooterSubsystem.setVelocity(ShooterConstants.TOP_MOTOR_SETPOINT_APM.in(RPM),
            ShooterConstants.BOTTOM_MOTOR_SEETPOINT_APM.in(RPM))));
    shooter.onFalse(new InstantCommand(() -> shooterSubsystem.setVelocity(0.0, 0.0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
