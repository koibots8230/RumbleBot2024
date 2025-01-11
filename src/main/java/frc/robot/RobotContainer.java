// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.Swerve;
import monologue.Logged;
import monologue.Monologue;

public class RobotContainer implements Logged {

  private XboxController controller;

  private final Swerve swerve;

  public RobotContainer(boolean isReal) {
    controller = new XboxController(0);

    swerve = new Swerve(isReal);

    Monologue.setupMonologue(this, "Robot", false, false);

    configureBindings();
    subsystemDefualtCommands();
  }

  private void configureBindings() {
  }

  private void subsystemDefualtCommands() {
    swerve.setDefaultCommand(
        swerve.fieldOrientedCommand(
            controller::getLeftY,
            controller::getLeftX,
            controller::getRightX));
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(15);
  }
}
