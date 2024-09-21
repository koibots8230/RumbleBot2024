package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motor.Motor;
import frc.lib.motor.MotorFactory;
import frc.robot.Constants;
import monologue.Annotations.Log;

public class Drivetrain extends SubsystemBase {
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;
  @Log private final SwerveModuleState[] moduleSetpoints;
  @Log private final SwerveModuleState[] moduleReal;
  @Log private final SwerveModulePosition[] modulePositionReal;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  public Drivetrain(boolean isReal) {
    // TODO: Set the default command, if any, for this subsystem by calling
    // setDefaultCommand(command)
    //       in the constructor or in the robot coordination class, such as RobotContainer.
    //       Also, you can call addChild(name, sendableChild) to associate sendables with the
    // subsystem
    //       such as SpeedControllers, Encoders, DigitalInputs, etc.

    frontLeftModule =
        new SwerveModule(
            Constants.DrivetrainConstants.FRONT_LEFT_DRIVE,
            Constants.DrivetrainConstants.FRONT_LEFT_TURN,
            isReal);
    frontRightModule =
        new SwerveModule(
            Constants.DrivetrainConstants.FRONT_RIGHT_DRIVE,
            Constants.DrivetrainConstants.FRONT_RIGHT_TURN,
            isReal);
    backLeftModule =
        new SwerveModule(
            Constants.DrivetrainConstants.BACK_LEFT_DRIVE,
            Constants.DrivetrainConstants.BACK_LEFT_TURN,
            isReal);
    backRightModule =
        new SwerveModule(
            Constants.DrivetrainConstants.BACK_RIGHT_DRIVE,
            Constants.DrivetrainConstants.BACK_RIGHT_TURN,
            isReal);

    moduleReal =
        new SwerveModuleState[] {
          frontLeftModule.getCurrentState(),
          frontRightModule.getCurrentState(),
          backLeftModule.getCurrentState(),
          backRightModule.getCurrentState()
        };
    moduleSetpoints =
        new SwerveModuleState[] {
          frontLeftModule.getSetpoint(),
          frontRightModule.getSetpoint(),
          backLeftModule.getSetpoint(),
          backRightModule.getSetpoint()
        };
    modulePositionReal =
        new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          backLeftModule.getPosition(),
          backRightModule.getPosition()
        };

    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(
                Constants.RobotConstants.LENGTH
                    .divide(2)
                    .minus(Constants.RobotConstants.WHEEL_OFFSET),
                Constants.RobotConstants.WIDTH
                    .divide(2)
                    .minus(Constants.RobotConstants.WHEEL_OFFSET)),
            new Translation2d(
                Constants.RobotConstants.LENGTH
                    .divide(2)
                    .minus(Constants.RobotConstants.WHEEL_OFFSET),
                Constants.RobotConstants.WIDTH
                    .divide(-2)
                    .minus(Constants.RobotConstants.WHEEL_OFFSET)),
            new Translation2d(
                Constants.RobotConstants.LENGTH
                    .divide(-2)
                    .minus(Constants.RobotConstants.WHEEL_OFFSET),
                Constants.RobotConstants.WIDTH
                    .divide(2)
                    .minus(Constants.RobotConstants.WHEEL_OFFSET)),
            new Translation2d(
                Constants.RobotConstants.LENGTH
                    .divide(-2)
                    .minus(Constants.RobotConstants.WHEEL_OFFSET),
                Constants.RobotConstants.WIDTH
                    .divide(-2)
                    .minus(Constants.RobotConstants.WHEEL_OFFSET)));

    odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromRadians(0), modulePositionReal);
  }

  public void set(ChassisSpeeds chassisSpeeds) {}

  public SwerveModuleState[] getModuleReal() {
    return moduleReal;
  }

  public SwerveModuleState[] getModuleSetpoints() {
    return moduleSetpoints;
  }

  public Pose2d getPose() {
    return new Pose2d();
    // TODO: Actually implement
  }

  @Override
  public void periodic() {
    moduleReal[0] = frontLeftModule.getCurrentState();
    moduleReal[1] = frontRightModule.getCurrentState();
    moduleReal[2] = backLeftModule.getCurrentState();
    moduleReal[3] = backRightModule.getCurrentState();

    moduleSetpoints[0] = frontLeftModule.getSetpoint();
    moduleSetpoints[1] = frontRightModule.getSetpoint();
    moduleSetpoints[2] = backLeftModule.getSetpoint();
    moduleSetpoints[3] = backRightModule.getSetpoint();

    modulePositionReal[0] = frontLeftModule.getPosition();
    modulePositionReal[1] = frontRightModule.getPosition();
    modulePositionReal[2] = backLeftModule.getPosition();
    modulePositionReal[3] = backRightModule.getPosition();
  }

  private static class SwerveModule {
    Motor driveMotor;
    Motor turnMotor;
    SwerveModuleState setpoint;

    SwerveModule(
        Motor.MotorDefinition driveMotor, Motor.MotorDefinition turnMotor, boolean isReal) {
      this.driveMotor = MotorFactory.get().create(driveMotor, isReal);
      this.turnMotor = MotorFactory.get().create(turnMotor, isReal);
    }

    Rotation2d getAngle() {
      return turnMotor.getPosition();
    }

    Measure<Velocity<Distance>> getVelocity() {
      return driveMotor.getVelocityDistance();
    }

    SwerveModuleState getCurrentState() {
      return new SwerveModuleState(getVelocity(), getAngle());
    }

    SwerveModuleState getSetpoint() {
      return setpoint;
    }

    SwerveModulePosition getPosition() {
      return new SwerveModulePosition(
          driveMotor.getPosition().getRotations() * Constants.RobotConstants.DRIVE_POSITION_FACTOR,
          getAngle());
    }

    void setState(SwerveModuleState state) {
      state = SwerveModuleState.optimize(state, getAngle()); // Direction optimization
      state.speedMetersPerSecond = state.speedMetersPerSecond / getAngle().getCos();
      setStateNoOptimized(state);
    }

    void setStateNoOptimized(SwerveModuleState state) {
      driveMotor.setVelocityDistance(Units.MetersPerSecond.of(state.speedMetersPerSecond));
      turnMotor.setPosition(Rotation2d.fromRadians(state.angle.getRadians()));
    }
  }
}
