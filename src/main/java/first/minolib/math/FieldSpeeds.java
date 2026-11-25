package first.minolib.math;

public class FieldSpeeds {
    /** Represents +X velocity w.r.t the field frame of reference. */
    public final double vx;
  
    /** Represents +Y velocity w.r.t the field frame of reference. */
    public final double vy;
  
    /** Represents the angular velocity of the field frame. (CCW is +) */
    public final double omega;
  
    /** Constructs a FieldSpeeds with zeros for dx, dy, and theta. */
    public FieldSpeeds() {
      this(0.0, 0.0, 0.0);
    }
  
    /**
     * Constructs a FieldSpeeds object.
     *
     * @param vx +X velocity.
     * @param vy +Y velocity.
     * @param omega Angular velocity.
     */
    public FieldSpeeds(final double vx, final double vy, final double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }
  
    @Override
    public String toString() {
        return "FieldSpeeds(" + vx + ", " + vy + ", " + omega + ")";
    }
}