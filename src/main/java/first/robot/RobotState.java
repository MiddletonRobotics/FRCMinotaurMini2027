package first.robot;

import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.IntSupplier;

import org.littletonrobotics.junction.Logger;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import first.minolib.math.ConcurrentTimeInterpolatableBuffer;
import first.minolib.vision.FieldPoseEstimate;

public class RobotState {
    private final Consumer<FieldPoseEstimate> visionEstimateConsumer;

    public RobotState(final Consumer<FieldPoseEstimate> visionEstimateConsumer) {
        this.visionEstimateConsumer = visionEstimateConsumer;
        fieldToRobot.addSample(0.0, Pose2d.ZERO);
        driveYawAngularVelocity.addSample(0.0, 0.0);
    }

    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot = ConcurrentTimeInterpolatableBuffer.createBuffer(Constants.kLoopBackTimeSeconds);

    private double visionConfidence = 0.0;
    private double lastUsedVisionEstimateTimestamp = 0.0;
    private Pose2d lastUsedVisionPoseEstimate = Pose2d.ZERO;

    private ChassisVelocities measuredRobotRelativeChassisVelocities = new ChassisVelocities();
    private ChassisVelocities measuredFieldRelativeChassisVelocities = new ChassisVelocities();
    private ChassisVelocities desiredRobotRelativeChassisVelocities = new ChassisVelocities();
    private ChassisVelocities desiredFieldRelativeChassisVelocities = new ChassisVelocities();
    private ChassisVelocities fusedFieldRelativeChassisVelocities = new ChassisVelocities();

    private int iteration = 0;

    private final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(Constants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(Constants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(Constants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularPosition = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(Constants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularPosition = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(Constants.kLoopBackTimeSeconds);

    private final ConcurrentTimeInterpolatableBuffer<Double> driveAccelerationX = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(Constants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveAccelerationY = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(Constants.kLoopBackTimeSeconds);

    private boolean enablePathCancel = false;

    private double autoStartTime;

    private Optional<Pose2d> trajectoryTargetPose = Optional.empty();
    private Optional<Pose2d> trajectoryCurrentPose = Optional.empty();

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        fieldToRobot.addSample(timestamp, pose);
    }

    public void setAutoStartTime(double timestamp) {
        autoStartTime = timestamp;
    }

    public double getAutoStartTime() {
        return autoStartTime;
    }

    public void enablePathCancel() {
        enablePathCancel = true;
    }

    public void disablePathCancel() {
        enablePathCancel = false;
    }

    public boolean getPathCancel() {
        return enablePathCancel;
    }

    public void incrementIterationCount() {
        iteration++;
    }

    public int getIteration() {
        return iteration;
    }

    public IntSupplier getIterationSupplier() {
        return () -> getIteration();
    }

    public void setTrajectoryTargetPose(Pose2d pose) {
        trajectoryTargetPose = Optional.of(pose);
    }

    public Optional<Pose2d> getTrajectoryTargetPose() {
        return trajectoryTargetPose;
    }

    public void setTrajectoryCurrentPose(Pose2d pose) {
        trajectoryCurrentPose = Optional.of(pose);
    }

    public Optional<Pose2d> getTrajectoryCurrentPose() {
        return trajectoryCurrentPose;
    }

    public double getDrivePitchRadians() {
        if (this.drivePitchAngularPosition.getInternalBuffer().lastEntry() != null) {
            return drivePitchAngularPosition.getInternalBuffer().lastEntry().getValue();
        }

        return 0.0;
    }

    public double getDriveRollRadians() {
        if (this.driveRollAngularPosition.getInternalBuffer().lastEntry() != null) {
            return driveRollAngularPosition.getInternalBuffer().lastEntry().getValue();
        }

        return 0.0;
    }

    public void addDriveMotionMeasurements(
        double timestamp,
        double yawAngularVelocity,
        double pitchAngularVelocity,
        double rollAngularVelocity,
        double pitchAngularPosition,
        double rollAngularPosition,
        double accelerationX,
        double accelerationY,
        ChassisVelocities measuredRobotRelativeChassisVelocities,
        ChassisVelocities measuredFieldRelativeChassisVelocities,
        ChassisVelocities desiredRobotRelativeChassisVelocities,
        ChassisVelocities desiredFieldRelativeChassisVelocities,
        ChassisVelocities fusedFieldRelativeChassisVelocities
    ) {
        driveYawAngularVelocity.addSample(timestamp, yawAngularVelocity);
        drivePitchAngularVelocity.addSample(timestamp, pitchAngularVelocity);
        driveRollAngularVelocity.addSample(timestamp, rollAngularVelocity);
        drivePitchAngularPosition.addSample(timestamp, pitchAngularPosition);
        driveRollAngularPosition.addSample(timestamp, rollAngularPosition);

        driveAccelerationX.addSample(timestamp, accelerationX);
        driveAccelerationY.addSample(timestamp, accelerationY);

        this.measuredRobotRelativeChassisVelocities = measuredRobotRelativeChassisVelocities;
        this.measuredFieldRelativeChassisVelocities = measuredFieldRelativeChassisVelocities;
        this.desiredRobotRelativeChassisVelocities = desiredRobotRelativeChassisVelocities;
        this.desiredFieldRelativeChassisVelocities = desiredFieldRelativeChassisVelocities;
        this.fusedFieldRelativeChassisVelocities = fusedFieldRelativeChassisVelocities;
    }

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        return fieldToRobot.getLatest();
    }

    public Pose2d getPredictedFieldToRobot(double lookaheadTimeSeconds) {
        var potentiallyFieldToRobot = getLatestFieldToRobot();
        Pose2d fieldToRobot = potentiallyFieldToRobot == null ? Pose2d.ZERO : potentiallyFieldToRobot.getValue();

        var delta = getLatestMeasuredRobotRelativeChassisVelocities().times(lookaheadTimeSeconds);

        // 2027: Pose2d.exp(Twist2d) was removed. Twist2d.exp() now returns a Transform2d to add to the pose.
        return fieldToRobot.plus(new Twist2d(delta.vx, delta.vy, delta.omega).exp());
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    public ChassisVelocities getLatestMeasuredRobotRelativeChassisVelocities() {
        return measuredRobotRelativeChassisVelocities;
    }

    public ChassisVelocities getLatestMeasuredFieldRelativeChassisVelocities() {
        return measuredFieldRelativeChassisVelocities;
    }

    public ChassisVelocities getLatestDesiredRobotRelativeChassisVelocities() {
        return desiredRobotRelativeChassisVelocities;
    }

    public ChassisVelocities getLatestDesiredFieldRelativeChassisVelocities() {
        return desiredFieldRelativeChassisVelocities;
    }

    public ChassisVelocities getLatestFusedFieldRelativeChassisVelocities() {
        return fusedFieldRelativeChassisVelocities;
    }

    public ChassisVelocities getLatestFusedRobotRelativeChassisVelocities() {
        var velocities = getLatestMeasuredRobotRelativeChassisVelocities();
        velocities.omega = getLatestFusedFieldRelativeChassisVelocities().omega;

        return velocities;
    }

    private Optional<Double> getMaxAbsoluteValueInRange(ConcurrentTimeInterpolatableBuffer<Double> buffer, double minTime, double maxTime) {
        var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
        var max = submap.stream().max(Double::compare);
        var min = submap.stream().min(Double::compare);

        if (max.isEmpty() || min.isEmpty()) {
            return Optional.empty();
        }

        if (Math.abs(max.get()) >= Math.abs(min.get())) return max;
        else return min;
    }

    public Optional<Double> getMaximumAbsoluteDriveYawAngularVelocityInRange(double minTime, double maxTime) {
        if (RobotBase.isReal()) {
            return getMaxAbsoluteValueInRange(driveYawAngularVelocity, minTime, maxTime);
        }

        return Optional.of(measuredRobotRelativeChassisVelocities.omega);
    }

    public Optional<Double> getMaximumAbsoluteDrivePitchAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsoluteValueInRange(drivePitchAngularVelocity, minTime, maxTime);
    }

    public Optional<Double> getMaximumAbsoluteDriveRollAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsoluteValueInRange(driveRollAngularVelocity, minTime, maxTime);
    }

    public void updateVisionPoseEstimate(FieldPoseEstimate weightedPoseEstimate) {
        lastUsedVisionEstimateTimestamp = weightedPoseEstimate.getTimestampSeconds();
        lastUsedVisionPoseEstimate = weightedPoseEstimate.getVisionRobotPoseMeters();
        visionEstimateConsumer.accept(weightedPoseEstimate);
    }

    public double lastUsedVisionTimestamp() {
        return lastUsedVisionEstimateTimestamp;
    }

    public Pose2d lastUsedVisionPose() {
        return lastUsedVisionPoseEstimate;
    }

    public void setVisionConfidence(double confidence) {
        // 2027: MathUtil.clamp was removed; Java's built-in Math.clamp does the same thing.
        visionConfidence = Math.clamp(confidence, 0.0, 1.0);
    }

    public double getVisionConfidence() {
        return visionConfidence;
    }

    public boolean isRedAlliance() {
        // 2027: DriverStation.getAlliance() moved to MatchState, and the enum values are now uppercase.
        return MatchState.getAlliance().equals(Optional.of(Alliance.RED));
    }

    public void updateLogger() {
        if (this.driveYawAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/YawAngularVelocity", this.driveYawAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.driveRollAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/RollAngularVelocity", this.driveRollAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.drivePitchAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/PitchAngularVelocity", this.drivePitchAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.drivePitchAngularPosition.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/PitchPositionRadians", this.drivePitchAngularPosition.getInternalBuffer().lastEntry().getValue());
        }
        if (this.driveRollAngularPosition.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/RollPositionRadians", this.driveRollAngularPosition.getInternalBuffer().lastEntry().getValue());
        }
        if (this.driveAccelerationX.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/AccelerationX", this.driveAccelerationX.getInternalBuffer().lastEntry().getValue());
        }
        if (this.driveAccelerationY.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/AccelerationY", this.driveAccelerationY.getInternalBuffer().lastEntry().getValue());
        }
        Logger.recordOutput("RobotState/DesiredChassisVelocitiesFieldFrame", getLatestDesiredFieldRelativeChassisVelocities());
        Logger.recordOutput("RobotState/DesiredChassisVelocitiesRobotFrame", getLatestDesiredRobotRelativeChassisVelocities());
        Logger.recordOutput("RobotState/MeasuredChassisVelocitiesFieldFrame", getLatestMeasuredFieldRelativeChassisVelocities());
        Logger.recordOutput("RobotState/MeasuredChassisVelocitiesRobotFrame", getLatestMeasuredRobotRelativeChassisVelocities());
        Logger.recordOutput("RobotState/FusedChassisVelocitiesFieldFrame", getLatestFusedFieldRelativeChassisVelocities());
    }
}
