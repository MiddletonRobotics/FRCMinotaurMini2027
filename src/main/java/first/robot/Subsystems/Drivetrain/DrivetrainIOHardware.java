package first.robot.Subsystems.Drivetrain;

import java.util.HashMap;
import java.util.Map;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import org.littletonrobotics.junction.Logger;

import first.minolib.utilities.PhoenixUtility;
import first.minolib.utilities.RobotTime;
import first.minolib.vision.FieldPoseEstimate;
import first.robot.RobotState;

public class DrivetrainIOHardware extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DrivetrainIO{
    private final RobotState robotState;
    private String[][] outputNames;

    private final String[] moduleNames = {"DrivetrainFL", "DrivetrainFR", "DrivetrainBL", "DrivetrainBR"};

    private final StatusSignal<Angle> pitch;
    private final StatusSignal<Angle> roll;

    private final StatusSignal<AngularVelocity> angularYawVelocity;
    private final StatusSignal<AngularVelocity> angularPitchVelocity;
    private final StatusSignal<AngularVelocity> angularRollVelocity;

    private final StatusSignal<LinearAcceleration> accelerationX;
    private final StatusSignal<LinearAcceleration> accelerationY;
    private final StatusSignal<LinearAcceleration> accelerationZ;


    @FunctionalInterface
    private interface ModuleInputUpdater {
        public void update(ModuleIOInputs moduleInputs, DrivetrainIOInputs drivetrainInputs);
    }

    private final Map<Integer, ModuleInputUpdater> moduleInputUpdaters = new HashMap<>();

    public DrivetrainIOHardware(RobotState robotState, SwerveDrivetrainConstants constants, @SuppressWarnings("unchecked") SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ... moduleConstants) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, constants, moduleConstants);
        this.robotState = robotState;

        for(int i = 0; i < moduleNames.length; i++) {
            final int moduleNumber = i;

            var module = getModule(moduleNumber);

            final TalonFX driveMotor = module.getDriveMotor();
            final TalonFX steerMotor = module.getSteerMotor();

            StatusSignal<Current> driveSupplyCurrent = driveMotor.getSupplyCurrent(false);
            StatusSignal<Current> driveStatorCurrent = driveMotor.getStatorCurrent(false);
            StatusSignal<Voltage> driveAppliedVoltage = driveMotor.getMotorVoltage(false);
            StatusSignal<Temperature> driveTemperature = driveMotor.getDeviceTemp(false);

            StatusSignal<AngularVelocity> steerVelocity = steerMotor.getVelocity(false);
            StatusSignal<Current> steerSupplyCurrent = steerMotor.getSupplyCurrent(false);
            StatusSignal<Current> steerStatorCurrent = steerMotor.getStatorCurrent(false);
            StatusSignal<Voltage> steerAppliedVoltage = steerMotor.getMotorVoltage(false);
            StatusSignal<Temperature> steerTemperature = steerMotor.getDeviceTemp(false);

            BaseStatusSignal.setUpdateFrequencyForAll(
                250.0, 
                driveSupplyCurrent,
                driveStatorCurrent,
                driveAppliedVoltage,
                driveTemperature,
                steerVelocity,
                steerSupplyCurrent,
                steerStatorCurrent,
                steerAppliedVoltage,
                steerTemperature
            );   

            PhoenixUtility.registerSignals(
                true, 
                driveSupplyCurrent,
                driveStatorCurrent,
                driveAppliedVoltage,
                driveTemperature,
                steerVelocity,
                steerSupplyCurrent,
                steerStatorCurrent,
                steerAppliedVoltage,
                steerTemperature
            );

            moduleInputUpdaters.put(moduleNumber, (moduleInputs, drivetrainInputs) -> {
                moduleInputs.driveConnected = BaseStatusSignal.isAllGood(driveSupplyCurrent, driveStatorCurrent, driveAppliedVoltage, driveTemperature);
                moduleInputs.driveSupplyCurrentAmperes = driveSupplyCurrent.getValueAsDouble();
                moduleInputs.driveStatorCurrentAmperes = driveStatorCurrent.getValueAsDouble();
                moduleInputs.driveAppliedVoltage = driveAppliedVoltage.getValueAsDouble();
                moduleInputs.driveTemperatureCelsius = driveTemperature.getValueAsDouble();

                moduleInputs.steerConnected = BaseStatusSignal.isAllGood(steerVelocity, steerSupplyCurrent, steerStatorCurrent, steerAppliedVoltage, steerTemperature);
                moduleInputs.steerPosition = drivetrainInputs.ModulePositions[moduleNumber].angle;
                moduleInputs.steerVelocity = Units.rotationsToRadians(steerVelocity.getValueAsDouble());
                moduleInputs.steerSupplyCurrentAmperes = steerSupplyCurrent.getValueAsDouble();
                moduleInputs.steerStatorCurrentAmperes = steerStatorCurrent.getValueAsDouble();
                moduleInputs.steerAppliedVoltage = steerAppliedVoltage.getValueAsDouble();
                moduleInputs.steerTemperatureCelsius = steerTemperature.getValueAsDouble();
            });
        }

        pitch = getPigeon2().getPitch();
        roll = getPigeon2().getRoll();

        angularYawVelocity = getPigeon2().getAngularVelocityZWorld();
        angularPitchVelocity = getPigeon2().getAngularVelocityYWorld();
        angularRollVelocity = getPigeon2().getAngularVelocityXWorld();

        accelerationX = getPigeon2().getAccelerationX();
        accelerationY = getPigeon2().getAccelerationY();
        accelerationZ = getPigeon2().getAccelerationZ();

        BaseStatusSignal.setUpdateFrequencyForAll(250.0, pitch, roll, angularYawVelocity, angularPitchVelocity, angularRollVelocity, accelerationX, accelerationY, accelerationZ);
        PhoenixUtility.registerSignals(true, pitch, roll, angularYawVelocity, angularPitchVelocity, angularRollVelocity, accelerationX, accelerationY, accelerationZ);
    }

    @Override
    public void updateInputs(DrivetrainIOInputs drivetrainInputs, ModuleIOInputs... moduleInputs) {
        drivetrainInputs.fromSwerveDriveState(this.getState());

        drivetrainInputs.yawVelocity = Units.degreesToRadians(angularYawVelocity.getValueAsDouble());
        drivetrainInputs.yawAcceleration = accelerationZ.getValueAsDouble() * 9.8067;

        for (int i = 0; i < moduleNames.length; i++) {
            moduleInputUpdaters.get(i).update(moduleInputs[i], drivetrainInputs);
        }

        ChassisVelocities measuredRobotRelativeChassisVelocities = getKinematics().toChassisVelocities(drivetrainInputs.ModuleVelocities);
        ChassisVelocities measuredFieldRelativeChassisVelocities = measuredRobotRelativeChassisVelocities.toFieldRelative(drivetrainInputs.Pose.getRotation());
        ChassisVelocities desiredRobotRelativeChassisVelocities = getKinematics().toChassisVelocities(drivetrainInputs.ModuleTargets);
        ChassisVelocities desiredFieldRelativeChassisSpeeds = desiredRobotRelativeChassisVelocities.toFieldRelative(drivetrainInputs.Pose.getRotation());
 
        ChassisVelocities fusedFieldRelativeChassisSpeeds = new ChassisVelocities(
            measuredFieldRelativeChassisVelocities.vx,
            measuredFieldRelativeChassisVelocities.vy,
            drivetrainInputs.yawVelocity
        );

        robotState.addDriveMotionMeasurements(
            RobotTime.getTimestampSeconds(),
            drivetrainInputs.yawVelocity,
            Units.degreesToRadians(angularPitchVelocity.getValueAsDouble()),
            Units.degreesToRadians(angularRollVelocity.getValueAsDouble()),
            Units.degreesToRadians(pitch.getValueAsDouble()),
            Units.degreesToRadians(roll.getValueAsDouble()),
            accelerationX.getValueAsDouble(),
            accelerationY.getValueAsDouble(),
            measuredRobotRelativeChassisVelocities,
            measuredFieldRelativeChassisVelocities,
            desiredRobotRelativeChassisVelocities,
            desiredFieldRelativeChassisSpeeds,
            fusedFieldRelativeChassisSpeeds
        );
    }

    @Override
    public void logModules(SwerveDriveState driveState) {
        if (driveState.ModuleVelocities == null) {
            return;
        }

        if (outputNames == null) {
            outputNames = new String[4][5];
            for (int i = 0; i < getModules().length; i++) {
                outputNames[i] = new String[5];
                outputNames[i][0] = moduleNames[i] + " Absolute Encoder Angle";
                outputNames[i][1] = moduleNames[i] + " Steering Angle";
                outputNames[i][2] = moduleNames[i] + " Target Steering Angle";
                outputNames[i][3] = moduleNames[i] + " Drive Velocity";
                outputNames[i][4] = moduleNames[i] + " Target Drive Velocity";
            }
        }
        for (int i = 0; i < getModules().length; i++) {
            Logger.recordOutput(outputNames[i][0], getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble() * 360);
            Logger.recordOutput(outputNames[i][1], driveState.ModulePositions[i].angle);
            Logger.recordOutput(outputNames[i][2], driveState.ModuleTargets[i].angle);
            Logger.recordOutput(outputNames[i][3], driveState.ModuleVelocities[i].velocity);
            Logger.recordOutput(outputNames[i][4], driveState.ModuleTargets[i].velocity);
        }
    }

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
    }

    @Override
    public void setSwerveRequest(SwerveRequest request) {
        super.setControl(request);
    }

    @Override
    public void resetRotation() {
        this.resetRotation(robotState.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.ZERO);
    }

    @Override
    public void resetToParameterizedRotation(Rotation2d rotation2d) {
        this.resetRotation(rotation2d);
    }

    @Override
    public void addVisionMeasurement(FieldPoseEstimate poseEstimate) {
        this.addVisionMeasurement(poseEstimate.getVisionRobotPoseMeters(), poseEstimate.getTimestampSeconds(), poseEstimate.getVisionMeasurementStdDevs());
    }
}


