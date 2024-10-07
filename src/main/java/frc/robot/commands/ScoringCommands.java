package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.DoubleSupplier;

public class ScoringCommands {

  public static Command scoreAmp(Elevator elevator, Indexer indexer) {
    return Commands.sequence(
        elevator.setPositionCommand(ElevatorConstants.AMP_POSITION),
        indexer.scoreAmp(),
        elevator.setPositionCommand(ElevatorConstants.SHOOTING_POSITION));
  }

  public static Command shootSpeaker(
      Elevator elevator,
      Indexer indexer,
      Shooter shooter,
      ShooterPivot shooterPivot,
      Swerve swerve,
      DoubleSupplier leftStick,
      DoubleSupplier rightStick) {
    return Commands.race(
        Commands.sequence(
            elevator.setPositionCommand(ElevatorConstants.SHOOTING_POSITION),
            Commands.parallel(
                indexer.alignForShot(),
                shooter.setVelocityCommand(
                    Constants.ShooterConstants.TOP_MOTOR_SETPOINT_SPEAKER,
                    Constants.ShooterConstants.TOP_MOTOR_SETPOINT_SPEAKER),
                shooterPivot.autoSetAngle(swerve.getOdometryPose())),
            indexer.shootSpeaker(),
            Commands.parallel(shooter.ShooterRestCommand())),
        swerve.fieldOrientedWhilePointingCommand(
            leftStick,
            rightStick));
  }
}
