package first.robot.Subsystems.Drivetrain;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;

import first.minolib.vision.FieldPoseEstimate;

public interface DrivetrainIO {
    @AutoLog

    public class DrivetrainIOInputs extends SwerveDriveState {
        public double yawVelocity = 0.0;
        public double yawAcceleration = 0.0;

        public double[] steerRates = new double[4];

        public DrivetrainIOInputs() {
            super(4);
        }

        public void fromSwerveDriveState(SwerveDriveState state) {
            this.Pose = state.Pose;
            this.RawHeading = state.RawHeading;
            this.ModuleVelocities = state.ModuleVelocities;
            this.ModuleTargets = state.ModuleTargets;
            this.ModulePositions = state.ModulePositions;
            this.Velocity = state.Velocity;
            this.SuccessfulDaqs = state.SuccessfulDaqs;
            this.FailedDaqs = state.FailedDaqs;
            this.OdometryPeriod = state.OdometryPeriod;
            this.Timestamp = state.Timestamp;
        }
    }

    @AutoLog

    public class ModuleIOInputs {
        public boolean driveConnected = false;
        public double driveSupplyCurrentAmperes = 0.0;
        public double driveStatorCurrentAmperes = 0.0;
        public double driveAppliedVoltage = 0.0;
        public double driveTemperatureCelsius = 0.0;

        public boolean steerConnected = false;
        public Rotation2d steerPosition = Rotation2d.ZERO;
        public double steerVelocity = 0.0;
        public double steerSupplyCurrentAmperes = 0.0;
        public double steerStatorCurrentAmperes = 0.0;
        public double steerAppliedVoltage = 0.0;
        public double steerTemperatureCelsius = 0.0;
    }

    /** Updates the inputs for the drivetrain */
    public void updateInputs(DrivetrainIOInputs drivetrainInputs, ModuleIOInputs... moduleInputs);

    /** Logs the state of the drivetrain modules for debugging */
    public void logModules(SwerveDriveState state);

    /** Applies commanded module states to the drivetrain */
    public default void setSwerveRequest(SwerveRequest request) {}

     /** Resets the drivetrain pose to a specific pose */
    public void resetPose(Pose2d pose);

    /** Resets the drivetrain rotation to Rotation2d.ZERO (forward) */
    public default void resetRotation() {}

    /** Resets the drivetrain rotation to a specific rotation */
    public default void resetToParameterizedRotation(Rotation2d rotation) {}

    public void addVisionMeasurement(FieldPoseEstimate poseEstimate);
}
