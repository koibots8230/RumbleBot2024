package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

import static edu.wpi.first.units.Units.*;

public class Drivetrain extends SubsystemBase implements Logged {
    static boolean isReal;
    SwerveModule[] swerveModules;
    Pigeon2 gyro;
    Rotation2d simYaw;
    SwerveDrivePoseEstimator odometry;

    @Log
    SwerveModuleState[] stateSetpoint;
    @Log
    SwerveModuleState[] stateReal;

    public PIDController xController;
    public PIDController yController;
    public PIDController thetaController;

    // private Field2d field = new Field2d();

    public Drivetrain(boolean isReal) {
        Drivetrain.isReal = isReal;

        swerveModules =
                new SwerveModule[] { // FL-FR-BL-BR
                        new SwerveModule(
                                DrivetrainConstants.DeviceIDs.FRONT_LEFT_DRIVE,
                                DrivetrainConstants.DeviceIDs.FRONT_LEFT_TURN,0),
                        new SwerveModule(
                                DrivetrainConstants.DeviceIDs.FRONT_RIGHT_DRIVE,
                                DrivetrainConstants.DeviceIDs.FRONT_RIGHT_TURN,1),
                        new SwerveModule(
                                DrivetrainConstants.DeviceIDs.BACK_LEFT_DRIVE,
                                DrivetrainConstants.DeviceIDs.BACK_LEFT_TURN,2),
                        new SwerveModule(
                                DrivetrainConstants.DeviceIDs.BACK_RIGHT_DRIVE,
                                DrivetrainConstants.DeviceIDs.BACK_RIGHT_TURN,3),
              };

        if (isReal) gyro = new Pigeon2(Constants.RobotConstants.GYRO_ID);
        simYaw = Rotation2d.fromRotations(0);

        odometry =
                new SwerveDrivePoseEstimator(
                        DrivetrainConstants.SWERVE_KINEMATICS,
                        isReal ? gyro.getRotation2d() : simYaw,
                        getModulePositions(),
                        new Pose2d());

        try (Notifier odometryUpdater =
            new Notifier(
                 () -> odometry
//                     .updateWithTime(
//                     //TODO: GET CURRENT TIME IN SECONDS
//                     0.0,
//                     isReal ? gyro.getRotation2d() : simYaw,
//                     getModulePositions())
                        .update(isReal ? gyro.getRotation2d() : simYaw, getModulePositions())
            )) {
//            odometryUpdater.startPeriodic(1.0 / 200); // Run at 200hz //TODO: FOR THE BETTER ODOMETRY
            odometryUpdater.startPeriodic(1.0 / 50); // Run at 200hz //TODO: FOR 50hz ODOMETRY (i think its default)
        }

        xController =
                new PIDController(
                        isReal ? DrivetrainConstants.VX_CONTROLLER_REAL.kp : DrivetrainConstants.VX_CONTROLLER_SIM.kp,
                        isReal ? DrivetrainConstants.VX_CONTROLLER_REAL.ki : DrivetrainConstants.VX_CONTROLLER_SIM.ki,
                        isReal ? DrivetrainConstants.VX_CONTROLLER_REAL.kd : DrivetrainConstants.VX_CONTROLLER_SIM.kd);
        yController =
                new PIDController(
                        isReal ? DrivetrainConstants.VY_CONTROLLER_REAL.kp : DrivetrainConstants.VY_CONTROLLER_SIM.kp,
                        isReal ? DrivetrainConstants.VY_CONTROLLER_REAL.ki : DrivetrainConstants.VY_CONTROLLER_SIM.ki,
                        isReal ? DrivetrainConstants.VY_CONTROLLER_REAL.kd : DrivetrainConstants.VY_CONTROLLER_SIM.kd);
        thetaController =
                new PIDController(
                        isReal ? DrivetrainConstants.VTHETA_CONTROLLER_REAL.kp : DrivetrainConstants.VTHETA_CONTROLLER_SIM.kp,
                        isReal ? DrivetrainConstants.VTHETA_CONTROLLER_REAL.ki : DrivetrainConstants.VTHETA_CONTROLLER_SIM.ki,
                        isReal ? DrivetrainConstants.VTHETA_CONTROLLER_REAL.kd : DrivetrainConstants.VTHETA_CONTROLLER_SIM.kd);

        SmartDashboard.putData("X Controller", xController);
        SmartDashboard.putData("Y Controller", yController);
        SmartDashboard.putData("Theta Controller", thetaController);

        stateReal = new SwerveModuleState[]{
                swerveModules[0].getState(),
                swerveModules[1].getState(),
                swerveModules[2].getState(),
                swerveModules[3].getState()
        };

        stateSetpoint = new SwerveModuleState[] {
            new SwerveModuleState(
                    swerveModules[0].inputs.driveVelocity, swerveModules[0].inputs.turnPosition),
                    new SwerveModuleState(
                            swerveModules[1].inputs.driveVelocity, swerveModules[0].inputs.turnPosition),
                    new SwerveModuleState(
                            swerveModules[2].inputs.driveVelocity, swerveModules[0].inputs.turnPosition),
                    new SwerveModuleState(
                            swerveModules[3].inputs.driveVelocity, swerveModules[0].inputs.turnPosition)
        };
    }

    @Override
    public void periodic() {
        odometry.update(isReal ? gyro.getRotation2d() : simYaw, getModulePositions());

        swerveModules[0].periodic();
        swerveModules[1].periodic();
        swerveModules[2].periodic();
        swerveModules[3].periodic();

        double[] statesDegrees = new double[8];
        double[] statesRadians = new double[8];
        for (int i = 0; i < 4; i++) {
            statesDegrees[i * 2] = swerveModules[i].getAngle().getDegrees();
            statesDegrees[(i * 2) + 1] = swerveModules[i].getVelocityMetersPerSec();
            statesRadians[i * 2] = swerveModules[i].getAngle().getRadians();
            statesRadians[(i * 2) + 1] = swerveModules[i].getVelocityMetersPerSec();
        }

        stateReal[0] = swerveModules[0].getState();
        stateReal[1] = swerveModules[1].getState();
        stateReal[2] = swerveModules[2].getState();
        stateReal[3] = swerveModules[3].getState();

        stateSetpoint[0] = new SwerveModuleState(
                swerveModules[0].inputs.driveVelocity, swerveModules[0].inputs.turnPosition);
        stateSetpoint[1] = new SwerveModuleState(
                swerveModules[1].inputs.driveVelocity, swerveModules[0].inputs.turnPosition);
        stateSetpoint[2] = new SwerveModuleState(
                swerveModules[2].inputs.driveVelocity, swerveModules[0].inputs.turnPosition);
        stateSetpoint[3] = new SwerveModuleState(
                swerveModules[3].inputs.driveVelocity, swerveModules[0].inputs.turnPosition);
    }

    public void addVisionMeasurement(Pose2d measurement, double timestamp) {
        odometry.addVisionMeasurement(measurement, timestamp);
    }

    public void zeroGyro() {
        if (isReal) gyro.reset();
        else simYaw = Rotation2d.fromRotations(0);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] targetModuleStates =
                DrivetrainConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
                targetModuleStates, RobotConstants.MAX_LINEAR_SPEED);

        if (speeds.vxMetersPerSecond == 0.0
                && speeds.vyMetersPerSecond == 0.0
                && speeds.omegaRadiansPerSecond == 0) {
            var currentStates = this.getModuleStates();
            targetModuleStates[0] = new SwerveModuleState(0, currentStates[0].angle);
            targetModuleStates[1] = new SwerveModuleState(0, currentStates[1].angle);
            targetModuleStates[2] = new SwerveModuleState(0, currentStates[2].angle);
            targetModuleStates[3] = new SwerveModuleState(0, currentStates[3].angle);
        }

        this.setModuleStates(targetModuleStates);
    }

    public void driveRobotRelativeByModule(ChassisSpeeds speeds, boolean[] whichModules) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] targetModuleStates =
                DrivetrainConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
                targetModuleStates, RobotConstants.MAX_LINEAR_SPEED);

        if (speeds.vxMetersPerSecond == 0.0
                && speeds.vyMetersPerSecond == 0.0
                && speeds.omegaRadiansPerSecond == 0) {
            var currentStates = this.getModuleStates();
            targetModuleStates[0] = new SwerveModuleState(0, currentStates[0].angle);
            targetModuleStates[1] = new SwerveModuleState(0, currentStates[1].angle);
            targetModuleStates[2] = new SwerveModuleState(0, currentStates[2].angle);
            targetModuleStates[3] = new SwerveModuleState(0, currentStates[3].angle);
        }

        for (int a = 0; a < 4; a++) {
            targetModuleStates[a] =
                    whichModules[a] ? targetModuleStates[a] : new SwerveModuleState();
        }

        this.setModuleStates(targetModuleStates);
    }

    public Rotation2d getGyroAngle() {
        return isReal ? gyro.getRotation2d() : simYaw;
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                swerveModules[0].getState(),
                swerveModules[1].getState(),
                swerveModules[2].getState(),
                swerveModules[3].getState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
        };
    }

    public ChassisSpeeds getRelativeSpeeds() {
        return DrivetrainConstants.SWERVE_KINEMATICS.toChassisSpeeds(this.getModuleStates());
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(isReal ? gyro.getRotation2d() : simYaw, getModulePositions(), pose);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        swerveModules[0].setState(states[0]);
        swerveModules[1].setState(states[1]);
        swerveModules[2].setState(states[2]);
        swerveModules[3].setState(states[3]);
    }

    public void stop() {
        swerveModules[0].stop();
        swerveModules[1].stop();
        swerveModules[2].stop();
        swerveModules[3].stop();
    }

    public void setCross() {
        setModuleStates(
                new SwerveModuleState[] {
                        new SwerveModuleState(0, new Rotation2d(0)), // Rotation2d.fromDegrees(45)),
                        new SwerveModuleState(0, new Rotation2d(0)), // Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0, new Rotation2d(0)), // Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0, new Rotation2d(0)), // Rotation2d.fromDegrees(45))
                });
    }

    public Pose2d getEstimatedPose() {
        return odometry.getEstimatedPosition();
    }

    public class SwerveModule {
        private final SwerveModuleInputs inputs = new SwerveModuleInputs();
        private final int index;

        private final SimpleMotorFeedforward driveFeedforward;
        private final PIDController driveFeedback;
        private final PIDController turnFeedback;
        private Rotation2d angleSetpoint =
                new Rotation2d(); // Setpoint for closed loop control, null for open loop
        private Double speedSetpoint = 0.0; // Setpoint for closed loop control, null for open loop

        private static final double LOOP_PERIOD_SECS = 0.02;

        private final DCMotorSim driveSim =
                new DCMotorSim(DCMotor.getNEO(1), RobotConstants.DRIVE_GEAR_RATIO, 0.025);
        private final DCMotorSim turnSim =
                new DCMotorSim(DCMotor.getNEO(1), RobotConstants.TURN_GEAR_RATIO, 0.004);

        private Measure<Voltage> driveAppliedVolts = Volts.of(0);
        private Measure<Voltage> turnAppliedVolts = Volts.of(0);

        private final CANSparkFlex driveMotor;
        private final CANSparkFlex turnMotor;
        private final RelativeEncoder driveEncoder;
        private final AbsoluteEncoder turnEncoder;
        private Rotation2d chassisAngularOffset;

        public static class SwerveModuleInputs {
            public Measure<Distance> drivePosition = Meters.of(0);
            public Measure<Velocity<Distance>> driveVelocity = MetersPerSecond.of(0);
            public Measure<Voltage> driveAppliedVoltage = Volts.of(0);
            public Measure<Current> driveCurrent = Amps.of(0);

            public Rotation2d turnPosition = new Rotation2d();
            public Measure<Velocity<Angle>> turnVelocity = RadiansPerSecond.of(0);
            public Measure<Voltage> turnAppliedVoltage = Volts.of(0);
            public Measure<Current> turnCurrent = Amps.of(0);
            public double setpoint = 0;
        }

        public SwerveModule(int driveId, int turnId, int index) {
            this.index = index;

            driveFeedforward =
                    new SimpleMotorFeedforward(
                            isReal ? DrivetrainConstants.DRIVE_FEEDFORWARD_REAL.ks : DrivetrainConstants.DRIVE_FEEDFORWARD_SIM.ks,
                            isReal ? DrivetrainConstants.DRIVE_FEEDFORWARD_REAL.kv : DrivetrainConstants.DRIVE_FEEDFORWARD_SIM.kv,
                            isReal ? DrivetrainConstants.DRIVE_FEEDFORWARD_REAL.ka : DrivetrainConstants.DRIVE_FEEDFORWARD_SIM.ka);
            driveFeedback =
                    new PIDController(
                            isReal ? DrivetrainConstants.DRIVE_PID_CONSTANTS_REAL.kp : DrivetrainConstants.DRIVE_PID_CONSTANTS_SIM.kp,
                            isReal ? DrivetrainConstants.DRIVE_PID_CONSTANTS_REAL.ki : DrivetrainConstants.DRIVE_PID_CONSTANTS_SIM.ki,
                            isReal ? DrivetrainConstants.DRIVE_PID_CONSTANTS_REAL.kd : DrivetrainConstants.DRIVE_PID_CONSTANTS_SIM.kd);
            turnFeedback =
                    new PIDController(
                            isReal ? DrivetrainConstants.TURN_PID_CONSTANTS_REAL.kp : DrivetrainConstants.TURN_PID_CONSTANTS_SIM.kp,
                            isReal ? DrivetrainConstants.TURN_PID_CONSTANTS_REAL.ki : DrivetrainConstants.TURN_PID_CONSTANTS_SIM.ki,
                            isReal ? DrivetrainConstants.TURN_PID_CONSTANTS_REAL.kd : DrivetrainConstants.TURN_PID_CONSTANTS_SIM.kd);

            turnFeedback.enableContinuousInput(0, 2 * Math.PI);

            driveFeedback.disableContinuousInput();

            SmartDashboard.putData("Swerve/Drive PID " + index, driveFeedback);
            SmartDashboard.putData("Swerve/Turn PID " + index, turnFeedback);

            turnFeedback.setTolerance(0.00001);

            /////////////////////////////////////////////////////////////////////////////////////////////////

            driveMotor = new CANSparkFlex(driveId, CANSparkLowLevel.MotorType.kBrushless);
            turnMotor = new CANSparkFlex(turnId, CANSparkLowLevel.MotorType.kBrushless);

            driveMotor.restoreFactoryDefaults();
            turnMotor.restoreFactoryDefaults();

            driveMotor.setSmartCurrentLimit(DrivetrainConstants.DRIVE.currentLimit);
            turnMotor.setSmartCurrentLimit(DrivetrainConstants.TURN.currentLimit);

            driveMotor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));
            turnMotor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));

            driveMotor.setInverted(DrivetrainConstants.DRIVE.inverted);
            turnMotor.setInverted(DrivetrainConstants.TURN.inverted);

            driveMotor.setIdleMode(DrivetrainConstants.DRIVE.idleMode);
            turnMotor.setIdleMode(DrivetrainConstants.TURN.idleMode);

            driveMotor.setCANTimeout((int) DrivetrainConstants.CAN_TIMEOUT.in(Millisecond));
            turnMotor.setCANTimeout((int) DrivetrainConstants.CAN_TIMEOUT.in(Millisecond));

            driveEncoder = driveMotor.getEncoder();
            turnEncoder = turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

            driveEncoder.setPositionConversionFactor(
                    DrivetrainConstants.DRIVING_ENCODER_POSITION_FACTOR.in(Meters));
            driveEncoder.setVelocityConversionFactor(
                    DrivetrainConstants.DRIVING_ENCODER_VELOCITY_FACTOR.in(MetersPerSecond));

            turnEncoder.setInverted(true);

            turnEncoder.setPositionConversionFactor(
                    DrivetrainConstants.TURNING_ENCODER_POSITION_FACTOR.in(Radians));
            turnEncoder.setVelocityConversionFactor(
                    DrivetrainConstants.TURNING_ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

            if (turnId == DrivetrainConstants.DeviceIDs.FRONT_LEFT_TURN) {
                chassisAngularOffset = Rotation2d.fromRadians((3 * Math.PI) / 2);
            } else if (turnId == DrivetrainConstants.DeviceIDs.FRONT_RIGHT_TURN) {
                chassisAngularOffset = new Rotation2d(Math.PI);
            } else if (turnId == DrivetrainConstants.DeviceIDs.BACK_LEFT_TURN) {
                chassisAngularOffset = Rotation2d.fromRadians(0);
            } else if (turnId == DrivetrainConstants.DeviceIDs.BACK_RIGHT_TURN) {
                chassisAngularOffset =
                        Rotation2d.fromRadians(
                                Math.PI / 2); // Rotation2d.fromDegrees((3 * Math.PI) / 2);
            }

            driveEncoder.setPosition(0.0);
            driveEncoder.setAverageDepth(DrivetrainConstants.DRIVE_ENCODER_SAMPLING_DEPTH);
        }

        public void periodic() {
            updateInputs(inputs);
            inputs.setpoint = angleSetpoint.getRadians();

            // Run closed loop turn control
            var turnPID = turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians());

            var angleOutput =
                    Volts.of(turnPID + (Math.signum(turnPID) * DrivetrainConstants.DRIVE_TURN_KS));
            //                         + (angleSetpoint.getRadians() -
            // Math.signum(getAngle().getRadians())) * ControlConstants.DRIVE_TURN_KS);

            setTurnVoltage(angleOutput);

            // Run closed loop drive control
            if (speedSetpoint > 0.1 || speedSetpoint < -0.1) {
                setDriveVoltage(
                        Volts.of(
                                driveFeedback.calculate(getVelocityMetersPerSec(), speedSetpoint)
                                        + driveFeedforward.calculate(speedSetpoint)));
            } else {
                // System.out.println("Zeroing voltage");
                setDriveVoltage(Volts.of(0));
            }
        }

        public SwerveModuleState setState(SwerveModuleState state) {

            // if (MathUtil.inputModulus(getAngle().minus(state.angle).getRadians(), -Math.PI, Math.PI)
            // >= Math.toRadians(90)) { // True if error is greater than 110 degrees TODO: Didn't work
            // Optimize state based on current angle
            var optimizedSetpoint = SwerveModuleState.optimize(state, getAngle());

            // Update setpoints, controllers run in "periodic"
            angleSetpoint = optimizedSetpoint.angle;
            speedSetpoint =
                    optimizedSetpoint.speedMetersPerSecond * Math.cos(turnFeedback.getPositionError());
            // Cosine scaling makes it so it won't drive (much) while module is turning

            return optimizedSetpoint;
            // } else {
            //     angleSetpoint = state.angle;
            //     speedSetpoint = state.speedMetersPerSecond;
            //     return state;
            // }
        }

        public void setVoltages(Measure<Voltage> driveVolts, Measure<Voltage> turnVolts) {
            setDriveVoltage(driveVolts);
            setTurnVoltage(turnVolts);
        }

        public void stop() {
            setTurnVoltage(Volts.of(0));
            setDriveVoltage(Volts.of(0));

            // Disable closed loop control for turn and drive
            angleSetpoint = getAngle();
            speedSetpoint = 0.0;
        }

        public Rotation2d getAngle() {
            return inputs.turnPosition;
        }

        /** Returns the current drive position of the module in meters. */
        public double getPositionMeters() {
            return inputs.drivePosition.in(Meters);
        }

        /** Returns the current drive velocity of the module in meters per second. */
        public double getVelocityMetersPerSec() {
            return inputs.driveVelocity.in(MetersPerSecond);
        }

        /** Returns the module position (turn angle and drive position). */
        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition(getPositionMeters(), getAngle());
        }

        /** Returns the module state (turn angle and drive velocity). */
        public SwerveModuleState getState() {
            return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
        }

        public void updateInputs(SwerveModuleInputs inputs) {
            if (isReal) {
                inputs.drivePosition = Meters.of(driveEncoder.getPosition());
                inputs.driveVelocity = MetersPerSecond.of(driveEncoder.getVelocity());
                inputs.driveAppliedVoltage =
                        Volts.of(driveMotor.getBusVoltage()).times(driveMotor.getAppliedOutput());
                inputs.driveCurrent = Amps.of(driveMotor.getOutputCurrent());

                inputs.turnPosition =
                        Rotation2d.fromRadians(turnEncoder.getPosition())
                                .plus(chassisAngularOffset)
                                .minus(Rotation2d.fromRadians(Math.PI));

                inputs.turnVelocity = RadiansPerSecond.of(turnEncoder.getVelocity());
                inputs.turnAppliedVoltage =
                        Volts.of(turnMotor.getBusVoltage()).times(turnMotor.getAppliedOutput());
                inputs.turnCurrent = Amps.of(turnMotor.getOutputCurrent());
            } else {
                driveSim.update(LOOP_PERIOD_SECS);
                turnSim.update(LOOP_PERIOD_SECS);

                inputs.drivePosition =
                        RobotConstants.DRIVE_WHEELS_RADIUS.times(driveSim.getAngularPositionRad());
                inputs.driveVelocity =
                        RobotConstants.DRIVE_WHEELS_RADIUS
                                .times(driveSim.getAngularVelocityRadPerSec())
                                .per(Second);
                inputs.driveAppliedVoltage = driveAppliedVolts;
                inputs.driveCurrent = Amps.of(driveSim.getCurrentDrawAmps());

                inputs.turnPosition =
                        new Rotation2d(turnSim.getAngularPositionRad() % (2 * Math.PI))
                                .minus(Rotation2d.fromRadians(Math.PI));
                inputs.turnVelocity = RadiansPerSecond.of(turnSim.getAngularVelocityRadPerSec());
                inputs.turnAppliedVoltage = turnAppliedVolts;
                inputs.turnCurrent = Amps.of(turnSim.getCurrentDrawAmps());
            }
        }

        public void setDriveVoltage(Measure<Voltage> voltage) {
            if (isReal) {
                driveMotor.setVoltage(voltage.in(Volts));
            } else {
                driveAppliedVolts = voltage;
                driveSim.setInputVoltage(driveAppliedVolts.in(Volts));
            }
        }

        public void setTurnVoltage(Measure<Voltage> voltage) {
            if (isReal) {
                turnMotor.setVoltage(voltage.in(Volts));
            } else {
                turnAppliedVolts = Volts.of(MathUtil.clamp(voltage.in(Volts), -12.0, 12.0));
                turnSim.setInputVoltage(turnAppliedVolts.in(Volts));
            }
        }
    }
}