package first.robot.Subsystems.Drivetrain;

import first.robot.Robot;

public class DrivetrainConstants {
    /**Maximum Achivable Linear Velocity Of Robot
     * Units: m/s */
    public static final double kMaxLinearVelocity = 6;
    /**Maximum Achivable Linear Acceleration Of Robot
     * Units: m/s^2 */
    public static final double kMaxLinearAcceleration = 6;
    /**Maximum Achivable Angular Velocity Of Robot
     * Units: rad/s */
    public static final double kMaximumRotationalVelocity = 3 * Math.PI;
    /**Maximum Achivable Angular Velocity Of Robot
     * Units: rad/s^2 */
    public static final double kMaxRoatationalAcceleration = 6 * Math.PI;

    public static final double kSkewCompensationSeconds = -0.03;
    //Drive To Pose PID Constants and Max Velocity/Acceleration
    public static final double kTranslationDriveToPosekP = Robot.isSimulation() ? 5.0 : 0.0;
    public static final double kTranslationDriveToPosekI = Robot.isSimulation() ? 0.0 : 0.0;
    public static final double kTranslationDriveToPosekD = Robot.isSimulation() ? 0.0 : 0.0;
    public static final double kTranslationDriveToPoseMaxVelocity = Robot.isSimulation() ? kMaxLinearVelocity : 0.0;
    public static final double kTranslationDriveToPoseMaxAcceleration = Robot.isSimulation() ? kMaxLinearAcceleration : 0.0;

    public static final double kRotationDriveToPosekP = Robot.isSimulation() ? 5.0 : 0.0;
    public static final double kRotationDriveToPosekI = Robot.isSimulation() ? 0.0 : 0.0;
    public static final double kRotationDriveToPosekD = Robot.isSimulation() ? 0.0 : 0.0;
    public static final double kRotationDriveToPoseMaxVelocity = Robot.isSimulation() ? kMaximumRotationalVelocity : 0.0;
    public static final double kRotationDriveToPoseMaxAcceleration = Robot.isSimulation() ? kMaxRoatationalAcceleration : 0.0;
}
