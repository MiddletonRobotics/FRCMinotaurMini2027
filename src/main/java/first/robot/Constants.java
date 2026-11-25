package first.robot;

import first.minolib.hardware.MinoCANBus;

public final class Constants {
    private Constants() {}

    /** How many seconds of history the RobotState buffers keep. */
    public static final double kLoopBackTimeSeconds = 1.0;

    /** When true, LoggedTunableNumbers can be changed from the dashboard. Same value as 2026. */
    public static final boolean kTuningMode = true;

    /** Driver Station port of the driver's controller. WPILib's 2027 Commands v3 template uses 1. */
    public static final int kDriverControllerPort = 1;

    // SystemCore's built-in CAN buses (it has can_s0 through can_s4; we use 0-3).
    // These replace 2026's kRioBus, since SystemCore has no single "rio" bus.
    public static final MinoCANBus kSystemCoreBus0 = new MinoCANBus("can_s0");
    public static final MinoCANBus kSystemCoreBus1 = new MinoCANBus("can_s1");
    public static final MinoCANBus kSystemCoreBus2 = new MinoCANBus("can_s2");
    public static final MinoCANBus kSystemCoreBus3 = new MinoCANBus("can_s3");
}
