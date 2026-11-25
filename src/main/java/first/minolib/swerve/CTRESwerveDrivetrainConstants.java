package first.minolib.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import first.robot.Constants;

public class CTRESwerveDrivetrainConstants {
    SwerveDrivetrainConstants driveTrainConstants;
    SwerveModuleConstants<?, ?, ?>[] moduleConstants;

    public CTRESwerveDrivetrainConstants(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        this.driveTrainConstants = driveTrainConstants;

        // 2027 MINIBOT: maple-sim is commented out for now, so always use the real module constants.
        // if (Constants.kUseMapleSim) {
        //     this.moduleConstants = MapleSimulatedSwerveDrivetrain.regulateModuleConstantsForSimulation(modules);
        // } else {
            this.moduleConstants = modules;
        // }
    }

    public SwerveDrivetrainConstants getDriveTrainConstants() {
        return driveTrainConstants;
    }

    public SwerveModuleConstants<?, ?, ?>[] getModuleConstants() {
        return moduleConstants;
    }
}