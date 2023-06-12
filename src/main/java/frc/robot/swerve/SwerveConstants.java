package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;

public interface SwerveConstants {
    double[] DrivePIDConstants = {0.5, 0, 0.02};
    double[] SteerPIDConstants = {0.5, 0, 0.02};
    double TicksToDegrees = 1.05;
    double TicksToMSP = 1.05;
    double TicksToMeters = 1.05;
    double ChassisDiameter = 0.5028943428;
    double MaxModuleMetersPerSecond = 4.572;
    double MaxRadiansPerSecond = MaxModuleMetersPerSecond / ChassisDiameter;
    static double nativeToDegrees(double n){
        return n * TicksToDegrees;
    }
    static double degreesToNative(double d){
        return d * (1.0 / TicksToDegrees);
    }
    static double MeterPerSecondToNative(double mps){
        return mps * TicksToMSP;
    }
    static double nativeToMetersPerSecond(double n) {
        return n * (1d / TicksToMSP);
    }
    static double nativeToMeters(double n){
        return n * TicksToMeters;
    }

    interface ModuleSettings{
        SwerveModuleSetting FrontLeft = new SwerveModuleSetting(
                0, 1, 0, 0d,
                ModuleOrdering.FrontLeft,
                new Translation2d(0.3556, 0.3556)
        );
        SwerveModuleSetting BackLeft = new SwerveModuleSetting(
                2, 3, 1, 0d,
                ModuleOrdering.BackLeft,
                new Translation2d(-0.3556, 0.3556)
        );
        SwerveModuleSetting FrontRight = new SwerveModuleSetting(
                4, 5, 2, 0d,
                ModuleOrdering.FrontRight,
                new Translation2d(0.3556, -0.3556)
        );
        SwerveModuleSetting BackRight = new SwerveModuleSetting(
                6, 7, 3, 0d,
                ModuleOrdering.BackRight,
                new Translation2d(-0.3556, -0.3556)
        );
    }
}
