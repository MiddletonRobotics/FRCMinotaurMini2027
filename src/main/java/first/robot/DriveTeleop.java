package first.robot;

import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

/**
 * The teleop OpMode. The {@code @Teleop} annotation makes it show up as a teleop option on the
 * Driver Station. The drivetrain's joystick driving is a default command set in Robot, so it runs
 * in any enabled mode; put teleop-only button bindings here.
 */
@Teleop
public class DriveTeleop implements OpMode {
    public DriveTeleop(Robot robot) {
        // Example: robot.driverController.a().onTrue(...);
    }
}
