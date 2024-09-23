package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;


public class DriveCommands extends Command {
    private final Drivetrain drivetrain;
    private final DoubleSupplier forward;
    private final DoubleSupplier sideways;
    private final DoubleSupplier rotation;

    public DriveCommands(Drivetrain drivetrain, DoubleSupplier forwards, DoubleSupplier sideways, DoubleSupplier rotation) {
        this.drivetrain = drivetrain;
        this.forward = forwards;
        this.sideways = sideways;
        this.rotation = rotation;
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
//        this.drivetrain.set(new ChassisSpeeds(
//                Constants.DriveConstants.MAX_VELOCITY.times(forward.getAsDouble()),
//                Constants.DriveConstants.MAX_VELOCITY.times(sideways.getAsDouble()),
//                Constants.DriveConstants.MAX_ROTATION.times(rotation.getAsDouble())
//        ));
    }

    @Override
    public boolean isFinished() {
//        return false;
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
