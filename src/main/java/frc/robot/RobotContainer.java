// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.Swerve;
import monologue.Logged;
import monologue.Monologue;

public class RobotContainer implements Logged {

  private XboxController controller;

  private final Swerve swerve;
  private final Intake intake;

  public RobotContainer(boolean isReal) {
    controller = new XboxController(0);

    intake = new Intake();

    swerve = new Swerve(isReal);

    Monologue.setupMonologue(this, "Robot", false, false);

    configureBindings();
    subsystemDefualtCommands();
  }

  private void configureBindings() {

    Trigger run = new Trigger(() -> controller.getRightTriggerAxis() > .15);
    run.onTrue(intake.IntakeCommand(Units.RPM.of(1000)));
    run.onFalse(intake.IntakeCommand(Units.RPM.of(0)));

    Trigger runrevese = new Trigger(() -> controller.getLeftTriggerAxis() > .15);
    runrevese.onTrue(intake.IntakeCommand(Units.RPM.of(-1000)));
    runrevese.onFalse(intake.IntakeCommand(Units.RPM.of(0)));

  }

  private void subsystemDefualtCommands() {
    swerve.setDefaultCommand(
        swerve.fieldOrientedCommand(
            (() -> -1 * controller.getLeftY()),
            (() -> -1 * controller.getLeftX()),
            (() -> -1 * controller.getRightX())));
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(15);
  }
}
