package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeCommands {
  public static Command intakeNote(Intake intake, Indexer indexer) {
    return Commands.race(
        indexer.intakeCommand(),
        intake.setVelocity(Constants.IntakeConstants.INTAKE_MOTOR_SETPOINT));
  }

  public static Command reverse(Intake intake, Indexer indexer) {
    return Commands.parallel(
        indexer.reverseIndexerCommand(
            Constants.IndexerConstants.TOP_REVERSE_SPEED,
            Constants.IndexerConstants.BOTTOM_REVERSE_SPEED),
        intake.setVelocity(Constants.IntakeConstants.INTAKE_MOTOR_REVERSE_SETPOINT));
  }

  public static Command stop(Intake intake, Indexer indexer) {
    return Commands.parallel(
        indexer.reverseIndexerCommand(
            Constants.IndexerConstants.TOP_MOTOR_STOP,
            Constants.IndexerConstants.BOTTOM_MOTOR_STOP),
        intake.setVelocity(Constants.IntakeConstants.INTAKE_MOTOR_STOP));
  }
}
