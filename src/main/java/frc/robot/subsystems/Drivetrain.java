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
import com.pathplanner.lib.util.PIDConstants;
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

import static edu.wpi.first.units.Units.*;

public class Drivetrain extends SubsystemBase {
    static boolean isReal;
    SwerveModule[] swerveModules;
    Pigeon2 gyro;
    SwerveDrivePoseEstimator odometry;

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

        gyro = new Pigeon2(Constants.RobotConstants.GYRO_ID);

        odometry =
                new SwerveDrivePoseEstimator(
                        DrivetrainConstants.SWERVE_KINEMATICS,
                        gyro.getRotation2d(),
                        getModulePositions(),
                        new Pose2d());

        try (Notifier odometryUpdater =
            new Notifier(
                 () -> odometry
//                     .updateWithTime(
//                     //TODO: GET CURRENT TIME IN SECONDS
//                     0.0,
//                     gyro.getRotation2d(),
//                     getModulePositions())
                        .update(gyro.getRotation2d(), getModulePositions())
            )) {
//            odometryUpdater.startPeriodic(1.0 / 200); // Run at 200hz //TODO: FOR THE BETTER ODOMETRY
            odometryUpdater.startPeriodic(1.0 / 50); // Run at 200hz //TODO: FOR 50hz ODOMETRY (i think its default)
        }

        xController =
                new PIDController(
                        DrivetrainConstants.VX_CONTROLLER.kP,
                        DrivetrainConstants.VX_CONTROLLER.kI,
                        DrivetrainConstants.VX_CONTROLLER.kD);
        yController =
                new PIDController(
                        DrivetrainConstants.VY_CONTROLLER.kP,
                        DrivetrainConstants.VY_CONTROLLER.kI,
                        DrivetrainConstants.VY_CONTROLLER.kD);
        thetaController =
                new PIDController(
                        DrivetrainConstants.VTHETA_CONTROLLER.kP,
                        DrivetrainConstants.VTHETA_CONTROLLER.kI,
                        DrivetrainConstants.VTHETA_CONTROLLER.kD);

        SmartDashboard.putData("X Controller", xController);
        SmartDashboard.putData("Y Controller", yController);
        SmartDashboard.putData("Theta Controller", thetaController);
    }

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), getModulePositions());

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
    }

    public void addVisionMeasurement(Pose2d measurement, double timestamp) {
        odometry.addVisionMeasurement(measurement, timestamp);
    }

    public void zeroGyro() {
        gyro.reset();
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
        return gyro.getRotation2d();
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
        odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
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
                            DrivetrainConstants.DRIVE_FEEDFORWARD_CONSTANTS.ks,
                            DrivetrainConstants.DRIVE_FEEDFORWARD_CONSTANTS.kv,
                            DrivetrainConstants.DRIVE_FEEDFORWARD_CONSTANTS.ka);
            driveFeedback =
                    new PIDController(
                            DrivetrainConstants.DRIVE_PID_CONSTANTS.kP,
                            DrivetrainConstants.DRIVE_PID_CONSTANTS.kI,
                            DrivetrainConstants.DRIVE_PID_CONSTANTS.kD);
            turnFeedback =
                    new PIDController(
                            DrivetrainConstants.TURN_PID_CONSTANTS.kP,
                            DrivetrainConstants.TURN_PID_CONSTANTS.kI,
                            DrivetrainConstants.TURN_PID_CONSTANTS.kD);

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

        /** Runs the module with the specified setpoint state. Returns the optimized state. */
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

        /** Disables all outputs to motors. */
        public void stop() {
            setTurnVoltage(Volts.of(0));
            setDriveVoltage(Volts.of(0));

            // Disable closed loop control for turn and drive
            angleSetpoint = getAngle();
            speedSetpoint = 0.0;
        }

        /** Returns the current turn angle of the module. */
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
                        RobotConstants.DRIVE_WHEELS.radius.times(driveSim.getAngularPositionRad());
                inputs.driveVelocity =
                        RobotConstants.DRIVE_WHEELS
                                .radius
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

    public static class PIDConstantsIO extends PIDConstants {
        public PIDConstantsIO(
                double realKp, double realKi, double realKd, double simKp, double simKi, double simKd) {
            super(
                    isReal ? realKp : simKp,
                    isReal ? realKi : simKi,
                    isReal ? realKd : simKd);
        }
    }

    public static class FeedforwardConstantsIO {
        public double ks = 0;
        public double kv = 0;
        public double ka = 0;
        public double kg = 0;

        public FeedforwardConstantsIO() {}

        public FeedforwardConstantsIO(double realKs, double realKv, double simKs, double simKv) {
            if (isReal) {
                this.ks = realKs;
                this.kv = realKv;
            } else {
                this.ks = simKs;
                this.kv = simKv;
            }
        }

        public FeedforwardConstantsIO(
                double realKs, double realKv, double realKa, double simKs, double simKv, double simKa) {
            if (isReal) {
                this.ks = realKs;
                this.kv = realKv;
                this.ka = realKa;
            } else {
                this.ks = simKs;
                this.kv = simKv;
                this.ka = simKa;
            }
        }
    }

    public static class Wheel {
        public Measure<Distance> circumference;
        public Measure<Distance> radius;

        public Wheel(Measure<Distance> radius) {
            this.radius = radius;
            circumference = radius.times(2 * Math.PI);
        }
    }

    public static class MotorConstantsIO {
        public final boolean inverted;
        public final int currentLimit;
        public final CANSparkBase.IdleMode idleMode;

        public MotorConstantsIO(boolean inverted, int currentLimit, CANSparkBase.IdleMode idleMode) {
            this.inverted = inverted;
            this.currentLimit = currentLimit;
            this.idleMode = idleMode;
        }

        public MotorConstantsIO(boolean inverted, int currentLimit) {
            this.inverted = inverted;
            this.currentLimit = currentLimit;
            this.idleMode = CANSparkBase.IdleMode.kCoast;
        }
    }
}