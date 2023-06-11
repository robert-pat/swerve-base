package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;

public interface SwerveConstants {
    interface ModuleSettings{
        SwerveModuleSetting FrontLeft = new SwerveModuleSetting(
                0, 0, 0, 0d,
                ModuleOrdering.FrontLeft,
                new Translation2d(0, 0)
        );
        SwerveModuleSetting BackLeft = new SwerveModuleSetting(
                0, 0, 0, 0d,
                ModuleOrdering.BackLeft,
                new Translation2d(0, 0)
        );
        SwerveModuleSetting FrontRight = new SwerveModuleSetting(
                0, 0, 0, 0d,
                ModuleOrdering.FrontRight,
                new Translation2d(0, 0)
        );
        SwerveModuleSetting BackRight = new SwerveModuleSetting(
                0, 0, 0, 0d,
                ModuleOrdering.BackRight,
                new Translation2d(0, 0)
        );
    }
    double MaxModuleMetersPerSecond = 5d;
}
