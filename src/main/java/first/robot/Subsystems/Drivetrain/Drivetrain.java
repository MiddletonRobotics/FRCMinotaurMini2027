package first.robot.Subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import first.minolib.advantagekit.LoggedTracer;
import first.minolib.advantagekit.LoggedTunableNumber;
import first.robot.RobotState;

public class Drivetrain implements Mechanism {
    private final RobotState robotState;

    private final DrivetrainIO io;
    private final DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();

    private final String[] moduleNames = {"DrivetrainFL", "DrivetrainFR", "DrivetrainBL", "DrivetrainBR"};

     private boolean isFieldCentric = true;

    private final ModuleIOInputsAutoLogged[] moduleInputs = new ModuleIOInputsAutoLogged[] { 
        new ModuleIOInputsAutoLogged(), 
        new ModuleIOInputsAutoLogged(), 
        new ModuleIOInputsAutoLogged(), 
        new ModuleIOInputsAutoLogged()
     };


    private final ChassisVelocities zeroChassisVelocities = new ChassisVelocities(0,0,0);

    private final SwerveRequest.FieldCentricFacingAngle continuousTracking = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
        .withHeadingPID(3.0, 0.0, 0.15) 
        .withDeadband(0.02);

    private final SwerveRequest.FieldCentric teleopRequestFC = new SwerveRequest.FieldCentric()
        .withDesaturateWheelVelocities(true)
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.RobotCentric teleopRequestRC = new SwerveRequest.RobotCentric()
        .withDesaturateWheelVelocities(true)
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    public final SwerveRequest.ApplyRobotVelocity robotVelocityRequest = new SwerveRequest.ApplyRobotVelocity()
        .withDesaturateWheelVelocities(true)
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.SwerveDriveBrake idleRequest = new SwerveRequest.SwerveDriveBrake()
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private static final LoggedTunableNumber driveToPoseTranslationkP = new LoggedTunableNumber("Drivetrain/DriveToPose/Translation/kP", DrivetrainConstants.kTranslationDriveToPosekP);
    private static final LoggedTunableNumber driveToPoseTranslationkI = new LoggedTunableNumber("Drivetrain/DriveToPose/Translation/kI", DrivetrainConstants.kTranslationDriveToPosekI);
    private static final LoggedTunableNumber driveToPoseTranslationkD = new LoggedTunableNumber("Drivetrain/DriveToPose/Translation/kD", DrivetrainConstants.kTranslationDriveToPosekD);
    private static final LoggedTunableNumber driveToPoseTranslationMaxVelocity = new LoggedTunableNumber("Drivetrain/DriveToPose/Translation/Max Velocity", DrivetrainConstants.kTranslationDriveToPoseMaxVelocity);
    private static final LoggedTunableNumber driveToPoseTranslationMaxAcceleration = new LoggedTunableNumber("Drivetrain/DriveToPose/Translation/Max Acceleration", DrivetrainConstants.kTranslationDriveToPoseMaxAcceleration);

    private static final LoggedTunableNumber driveToPoseRotationkP = new LoggedTunableNumber("Drivetrain/DriveToPose/Rotation/kP", DrivetrainConstants.kRotationDriveToPosekP);
    private static final LoggedTunableNumber driveToPoseRotationkI = new LoggedTunableNumber("Drivetrain/DriveToPose/Rotation/kI", DrivetrainConstants.kRotationDriveToPosekI);
    private static final LoggedTunableNumber driveToPoseRotationkD = new LoggedTunableNumber("Drivetrain/DriveToPose/Rotation/kD", DrivetrainConstants.kRotationDriveToPosekD);
    private static final LoggedTunableNumber driveToPoseRotationMaxVelocity = new LoggedTunableNumber("Drivetrain/DriveToPose/Rotation/Max Velocity", DrivetrainConstants.kRotationDriveToPoseMaxVelocity);
    private static final LoggedTunableNumber driveToPoseRotationMaxAcceleration = new LoggedTunableNumber("Drivetrain/DriveToPose/Rotation/Max Acceleration", DrivetrainConstants.kRotationDriveToPoseMaxAcceleration);


    public Drivetrain(RobotState robotState, DrivetrainIO io) {
        this.robotState = robotState;
        this.io = io;

        continuousTracking.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic() {
        io.updateInputs(inputs, moduleInputs);
        io.logModules(inputs);

        for(int i = 0; i < moduleInputs.length; i++ ) {
            Logger.processInputs("drivetrain", inputs);
            LoggedTracer.record("DrivetrainPeriodic");
        }
    }

    
    public Command drive(DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier rotation, BooleanSupplier isFieldCentric) {
        return runRepeatedly(() -> {
            ChassisVelocities velocities = calcualteVelocities(throttle, strafe, rotation);

            boolean fieldCentric = isFieldCentric.getAsBoolean();
            
            if (fieldCentric) {
                applyRequest(teleopRequestFC
                    .withVelocityX(velocities.vx)
                    .withVelocityY(velocities.vy)
                    .withRotationalRate(velocities.omega)
                );
            } else {
                applyRequest(teleopRequestRC
                    .withVelocityX(velocities.vx)
                    .withVelocityY(velocities.vy)
                    .withRotationalRate(velocities.omega)
                );
            }
        })
        .whenExited(this::stop)
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Drivetrain.ControllerDrive");
    }

    /** Drives to a fixed pose. Shortcut for {@link #driveToPose(Supplier)}. */
    public Command driveToPose(Pose2d target) {
        return driveToPose(() -> target);
    }

    
    public Command driveToPose(Supplier<Pose2d> targetSupplier) {
        return run(coroutine -> {
            ProfiledPIDController translationController = new ProfiledPIDController(
                driveToPoseTranslationkP.get(),
                driveToPoseTranslationkI.get(),
                driveToPoseTranslationkD.get(),
                new TrapezoidProfile.Constraints(
                    driveToPoseTranslationMaxVelocity.get(),
                    driveToPoseTranslationMaxAcceleration.get()
                )
            );

            ProfiledPIDController rotationController = new ProfiledPIDController(
                driveToPoseRotationkP.get(),
                driveToPoseRotationkI.get(),
                driveToPoseRotationkD.get(),
                new TrapezoidProfile.Constraints(
                    driveToPoseRotationMaxVelocity.get(),
                    driveToPoseRotationMaxAcceleration.get()
                )
            );

            rotationController.enableContinuousInput(-Math.PI, Math.PI);

            Pose2d startPose = robotState.getLatestFieldToRobot().getValue();
            ChassisVelocities startVelocities = robotState.getLatestMeasuredFieldRelativeChassisVelocities();

            Translation2d startError = targetSupplier.get().minus(startPose).getTranslation();
            Rotation2d startDirection = startError.getAngle().orElse(Rotation2d.ZERO);

            double velocityTowardTarget = startVelocities.vx * startDirection.getCos() + startVelocities.vy * startDirection.getSin();

            translationController.reset(startError.getNorm(), -velocityTowardTarget);
            rotationController.reset(startPose.getRotation().getRadians(), startVelocities.omega);

            while (!isAtPose(targetSupplier.get())) {
                // Pick up gain changes made on the dashboard
                if (driveToPoseTranslationkP.hasChanged(hashCode()) || driveToPoseTranslationkI.hasChanged(hashCode()) || driveToPoseTranslationkD.hasChanged(hashCode())) {
                    translationController.setPID(driveToPoseTranslationkP.get(), driveToPoseTranslationkI.get(), driveToPoseTranslationkD.get());
                }
                if (driveToPoseRotationkP.hasChanged(hashCode()) || driveToPoseRotationkI.hasChanged(hashCode()) || driveToPoseRotationkD.hasChanged(hashCode())) {
                    rotationController.setPID(driveToPoseRotationkP.get(), driveToPoseRotationkI.get(), driveToPoseRotationkD.get());
                }
                if (driveToPoseTranslationMaxVelocity.hasChanged(hashCode()) || driveToPoseTranslationMaxAcceleration.hasChanged(hashCode())) {
                    translationController.setConstraints(new TrapezoidProfile.Constraints(driveToPoseTranslationMaxVelocity.get(), driveToPoseTranslationMaxAcceleration.get()));
                }
                if (driveToPoseRotationMaxVelocity.hasChanged(hashCode()) || driveToPoseRotationMaxAcceleration.hasChanged(hashCode())) {
                    rotationController.setConstraints(new TrapezoidProfile.Constraints(driveToPoseRotationMaxVelocity.get(), driveToPoseRotationMaxAcceleration.get()));
                }

                Pose2d targetPose = targetSupplier.get();
                Pose2d currentPose = robotState.getLatestFieldToRobot().getValue();

                Translation2d translationError = targetPose.minus(currentPose).getTranslation();
                Rotation2d directionToTarget = translationError.getAngle().orElse(Rotation2d.ZERO);

                double translationOutput = translationController.calculate(translationError.getNorm(), 0.0);
                double rotationOutput = rotationController.calculate(
                    currentPose.getRotation().getRadians(),
                    targetPose.getRotation().getRadians()
                );

                Translation2d translationVelocity = new Translation2d(-translationOutput, directionToTarget);

                applyRequest(robotVelocityRequest.withVelocity(
                    new ChassisVelocities(translationVelocity.getX(), translationVelocity.getY(), rotationOutput)
                ));

                Logger.recordOutput("DriveToPose/Target Pose", targetPose);
                Logger.recordOutput("DriveToPose/Translation Output", translationOutput);
                Logger.recordOutput("DriveToPose/Rotation Output", rotationOutput);
                Logger.recordOutput("DriveToPose/Translation Error", translationError.getNorm());
                Logger.recordOutput("DriveToPose/Rotation Error", targetPose.getRotation().minus(currentPose.getRotation()).getRadians());

                coroutine.yield(); 
            }
        })
        .whenExited(this::stop)
        .named("Drivetrain.DriveToPose");
    }

    private boolean isAtPose(Pose2d target) {
        Pose2d current = robotState.getLatestFieldToRobot().getValue();
        double tolerance = Units.inchesToMeters(0.5);
        return MathUtil.isNear(target.getX(), current.getX(), tolerance)
            && MathUtil.isNear(target.getY(), current.getY(), tolerance);
    }

    public void applyRequest(SwerveRequest request) {
        io.setSwerveRequest(request);
    }

    public void stop() {
        applyRequest(robotVelocityRequest.withVelocity(zeroChassisVelocities));
    }


    private ChassisVelocities calcualteVelocities(DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier omega) {
        if(MatchState.getAlliance().isEmpty()) {
            return zeroChassisVelocities;
        }

        double magnitudeX = MathUtil.applyDeadband(throttle.getAsDouble(), 0.1); //TODO put in constants class
        double magnitudeY = MathUtil.applyDeadband(strafe.getAsDouble(), 0.1);
        double magnitudeTheta = MathUtil.applyDeadband(omega.getAsDouble(), 0.1);

        double velocityX = magnitudeX * DrivetrainConstants.kMaxLinearVelocity;
        double velocityY = magnitudeY * DrivetrainConstants.kMaxLinearVelocity;
        double velocityTheta = magnitudeTheta * DrivetrainConstants.kMaximumRotationalVelocity;

        Rotation2d fieldRelativeHeading = robotState.getLatestFieldToRobot().getValue().getRotation();

          Rotation2d skewCompensationFactor = Rotation2d.fromRadians(robotState.getLatestMeasuredRobotRelativeChassisVelocities().omega * DrivetrainConstants.kSkewCompensationSeconds);

        return new ChassisVelocities(velocityX, velocityY, velocityTheta)
            .toRobotRelative(fieldRelativeHeading)
            .toFieldRelative(fieldRelativeHeading.plus(skewCompensationFactor));
    }
}
