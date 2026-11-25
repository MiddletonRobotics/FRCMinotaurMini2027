package first.minolib.vision;

import org.wpilib.math.geometry.Pose3d;

/** A data class for a fiducial result. */
public record PhotonFiducialResult(
    double timestamp,
    Pose3d cameraPose,
    double latencySecs,
    double averageAmbiguity,
    double reprojectionError,
    long tagsSeenBitMap,
    int numTags,
    double averageTagDistance,
    PoseObservationType type
) {}