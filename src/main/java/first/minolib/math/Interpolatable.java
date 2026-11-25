package first.minolib.math;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Twist2d;

/**
 * The Interpolatable class provides a framework for creating objects that can be interpolated.
 *
 * @param <T> The type of the value that can be interpolated.
 */
public abstract class Interpolatable<T> {
    private final T value;

    public Interpolatable(T value) {
        this.value = value;
    }

    public T get() {
        return this.value;
    }

    public abstract T interpolate(T endValue, double t);

    public T interpolate(Interpolatable<T> endValue, double t) {
        return interpolate(endValue.value, t);
    }

    public static Interpolatable<Double> interDouble(double value) {
        return new Interpolatable<Double>(value) {
            @Override
            public Double interpolate(Double endValue, double t) {
                if (t < 0) {
                return this.get();
                } else if (t >= 1) {
                return endValue;
                } else {
                return (1 - t) * super.value + t * endValue; // Lerp
                }
            }
        };
    }

    public static Interpolatable<Pose2d> interPose2d(Pose2d value) {
        return new Interpolatable<Pose2d>(value) {
            @Override
            public Pose2d interpolate(Pose2d endValue, double t) {
                if (t < 0) {
                return this.get();
                } else if (t >= 1) {
                return endValue;
                } else {
                var twist = endValue.minus(super.value).log(); // 2027: Pose2d.log(end) -> end.minus(start).log()
                return new Pose2d().plus(new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t).exp()); // 2027: pose.exp(twist) -> pose.plus(twist.exp())
                }
            }
        };
    }
}