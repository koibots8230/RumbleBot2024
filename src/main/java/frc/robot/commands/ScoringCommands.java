package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.*;

public class ScoringCommands {
    
    public static Command scoreAmp(Elevator elevator, Indexer indexer) {
        return Commands.sequence(
            elevator.setPositionCommand(ElevatorConstants.AMP_POSITION),
            indexer.scoreAmp(),
            elevator.setPositionCommand(ElevatorConstants.SHOOTING_POSITION)
        );
    }

    public static Command shootSpeaker(Elevator elevator, Indexer indexer, Shooter shooter) { //TODO: Add pivot
        return Commands.sequence(
            elevator.setPositionCommand(ElevatorConstants.SHOOTING_POSITION),
            Commands.parallel(
                indexer.alignForShot()
                //TODO: Add spinup shooter command
                //TODO: Add change shooter angle command
            ),
            indexer.shootSpeaker(),
            Commands.parallel(
                //TODO: Add spin down shooter command
                //TODO: Add reset shooter angle command11
            )
        );
    }
}
