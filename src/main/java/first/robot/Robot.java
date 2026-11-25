package first.robot;

import org.littletonrobotics.junction.LoggedOpModeRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;

import first.minolib.advantagekit.LoggedTracer;
import first.robot.Subsystems.Drivetrain.Drivetrain;
import first.robot.Subsystems.Drivetrain.DrivetrainIO;

/**
 * The main robot class. LoggedOpModeRobot is AdvantageKit's version of the Commands v3
 * OpModeRobot: it runs the AdvantageKit logger every loop and automatically finds any
 * {@code @Teleop} / {@code @Autonomous} classes in this package.
 */
public class Robot extends LoggedOpModeRobot {
    final RobotState robotState;
    final Drivetrain drivetrain;

    final CommandGamepad driverController = new CommandGamepad(Constants.kDriverControllerPort);

    private final Scheduler scheduler = Scheduler.getDefault();

    public Robot() {
        // ---- AdvantageKit logging setup (must happen before Logger.start()) ----
        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // save a .wpilog file on the robot
        }
        Logger.addDataReceiver(new NT4Publisher()); // live data for AdvantageScope
        Logger.start();

        robotState = new RobotState(visionEstimate -> {});

        // TODO replace this placeholder with a real DrivetrainIOHardware once it exists
        DrivetrainIO drivetrainIO = new DrivetrainIO() {
            @Override
            public void updateInputs(DrivetrainIO.DrivetrainIOInputs drivetrainInputs, DrivetrainIO.ModuleIOInputs... moduleInputs) {}

            @Override
            public void logModules(SwerveDriveState state) {}

            @Override
            public void setSwerveRequest(SwerveRequest request) {}
        };
        drivetrain = new Drivetrain(robotState, drivetrainIO);

        drivetrain.setDefaultCommand(
            drivetrain.drive(
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> true));
    }

    @Override
    public void robotPeriodic() {
        LoggedTracer.reset();

        drivetrain.periodic();

        scheduler.run();
        LoggedTracer.record("Commands");

        robotState.updateLogger();
        LoggedTracer.record("RobotPeriodic");
    }
}
