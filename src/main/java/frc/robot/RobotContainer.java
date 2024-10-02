// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.Swerve;
import monologue.Logged;
import monologue.Monologue;

public class RobotContainer implements Logged {

  private XboxController controller;

  private final Elevator elevator;
  private final ShooterPivot shooterPivot;
  private final Shooter shooter;
  private final Swerve swerve;
  private final Vision vision;

  public RobotContainer(boolean isReal) {
    controller = new XboxController(0);

    elevator = new Elevator(isReal);

    shooterPivot = new ShooterPivot();

    shooter = new Shooter(isReal);

    swerve = new Swerve(isReal);

    vision =
        new Vision(swerve::getOdometryPose, swerve::getGyroAngle, swerve::addVisionMeasurement);

    Monologue.setupMonologue(this, "Robot", false, false);

    configureBindings();
    subsystemDefualtCommands();
  }

  private void configureBindings() {

    Trigger elevatorUp = new Trigger(() -> controller.getRawButton(1));
    elevatorUp.onTrue(elevator.setPositionCommand(Inches.of(3)));
    elevatorUp.onFalse(elevator.setPositionCommand(Inches.of(0)));

    Trigger setAngle = new Trigger(() -> controller.getRawButton(2));
    setAngle.onTrue(shooterPivot.setPositionCommand(Rotation2d.fromDegrees(42)));
    setAngle.onFalse(shooterPivot.setPositionCommand(Rotation2d.fromDegrees(0)));
  }

  private void subsystemDefualtCommands() {

    shooterPivot.setDefaultCommand(
        shooterPivot.setPositionCommand(Constants.ShooterPivotConstants.REST_POSITION));

    shooter.setDefaultCommand(shooter.setVelocityCommand(100, 100));

    swerve.setDefaultCommand(
        swerve.fieldOrientedCommand(
            controller::getLeftY, controller::getLeftX, controller::getRightX));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
