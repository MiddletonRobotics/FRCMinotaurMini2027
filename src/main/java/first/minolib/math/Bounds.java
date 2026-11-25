package first.minolib.math;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Translation2d;

public record Bounds(double minX, double maxX, double minY, double maxY) {
  public boolean contains(Translation2d translation) {
    return translation.getX() >= minX()
        && translation.getX() <= maxX()
        && translation.getY() >= minY()
        && translation.getY() <= maxY();
    }

    public Translation2d clamp(Translation2d translation) {
        return new Translation2d(Math.clamp(translation.getX(), minX(), maxX()), Math.clamp(translation.getY(), minY(), maxY()));
    }
}