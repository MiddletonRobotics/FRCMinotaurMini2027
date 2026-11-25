// 2027 MINIBOT: Commented out. ChassisVelocities now implements Interpolatable itself, so this class is no longer needed.
// package first.minolib.math;
// 
// import org.wpilib.math.interpolation.Interpolatable;
// import org.wpilib.math.kinematics.ChassisVelocities;
// 
// public class InterpolateableChassisSpeeds extends ChassisVelocities implements Interpolatable<InterpolateableChassisSpeeds> {
//     public InterpolateableChassisSpeeds(double vx, double vy, double omega) {
//         super(vx, vy, omega);
//     }
// 
//     @Override
//     public InterpolateableChassisSpeeds interpolate(InterpolateableChassisSpeeds endValue, double t) {
//         if (t < 0) {
//             return this;
//         }
//         if (t >= 1) {
//             return endValue;
//         }
// 
//         return new InterpolateableChassisSpeeds(
//             (1 - t) * vx + t * endValue.vx,
//             (1 - t) * vy + t * endValue.vy,
//             (1 - t) * omega + t * endValue.omega);
//     }
// 
//     public static InterpolateableChassisSpeeds fromChassisSpeeds(ChassisVelocities cs) {
//         return new InterpolateableChassisSpeeds(cs.vx, cs.vy, cs.omega);
//     }
// }