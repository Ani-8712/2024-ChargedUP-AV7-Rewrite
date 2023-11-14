package team3647.lib;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, 250, modules);
    }

    public Pigeon2 getGyro() {
        return super.m_pigeon2;
    }
}
