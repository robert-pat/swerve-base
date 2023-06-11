package frc.robot.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;

import static frc.robot.swerve.SwerveConstants.MaxModuleMetersPerSecond;
import static frc.robot.swerve.SwerveConstants.ModuleSettings.*;
/**A 4 module swerve drive. Uses Falcon500s for both the drive and steer motors.
 * NOTE: Certain methods <b>MUST</b> be called for proper functionality: <br>
 * init() <br>
 * updateOdometry()
 */
@SuppressWarnings("unused")
public class SwerveDrive {
    private final SwerveModule[] modules = {
            new SwerveModule(SwerveConstants.ModuleSettings.FrontLeft),
            new SwerveModule(SwerveConstants.ModuleSettings.BackLeft),
            new SwerveModule(SwerveConstants.ModuleSettings.FrontRight),
            new SwerveModule(SwerveConstants.ModuleSettings.BackRight)
    };
    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            FrontLeft.Position,
            BackLeft.Position,
            FrontRight.Position,
            BackRight.Position
    );
    public final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            kinematics,
            new Rotation2d(),
            new SwerveModulePosition[4]
    );
    public final AHRS gyro = new AHRS();

    @SuppressWarnings("unused")
    public void init(){
        odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), new Pose2d());
        for (SwerveModule s : modules){
            s.init();
        }
    }
    @SuppressWarnings("unused")
    public void resetAllOdometry(Pose2d fieldPos){
        odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), fieldPos);
    }
    /**Commands the robot to drive
     * @param speeds the desired speeds
     * @param isFieldRelative whether the speeds are relative to the field
     */
    @SuppressWarnings("unused")
    public void driveSwerve(ChassisSpeeds speeds, boolean isFieldRelative){
        if (isFieldRelative){
            driveFieldRelative(speeds);
        }
        else{
            driveRobotRelative(speeds);
        }
    }
    /**[WIP- field relative custom centers are untested / may be handled incorrectly] <br>
     * Sets the target state of the swerve drive, with a custom center of rotation.
     * @param speeds the desired speeds, either robot or field-relative
     * @param center the center of rotation; what omega velocity will rotate around
     * @param isFieldRelative whether the speeds are field relative or not
     */
    @SuppressWarnings("unused")
    public void driveSwerveAboutPoint(ChassisSpeeds speeds, Translation2d center, boolean isFieldRelative){
        if (isFieldRelative){
            Translation2d v = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
            v = v.rotateBy(gyro.getRotation2d().unaryMinus());
            speeds = new ChassisSpeeds(
                    v.getX(),
                    v.getY(),
                    speeds.omegaRadiansPerSecond
            );
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds, center);
            applyUnadjustedModuleStates(states);
        }
        else{
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds, center);
            applyUnadjustedModuleStates(states);
        }
    }
    protected void driveRobotRelative(ChassisSpeeds speeds){
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        applyUnadjustedModuleStates(states);
    }
    protected void driveFieldRelative(ChassisSpeeds speeds){
        Translation2d v = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        v = v.rotateBy(gyro.getRotation2d().unaryMinus());
        speeds = new ChassisSpeeds(
                v.getX(),
                v.getY(),
                speeds.omegaRadiansPerSecond
        );
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        applyUnadjustedModuleStates(states);
    }
    protected void applyUnadjustedModuleStates(SwerveModuleState[] states){
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MaxModuleMetersPerSecond);
        for (ModuleOrdering ord : ModuleOrdering.values()){
            modules[ord.ordinal()].setTargetState(states[ord.ordinal()]);
        }
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (ModuleOrdering ord : ModuleOrdering.values()){
            positions[ord.ordinal()] = modules[ord.ordinal()].getPosition();
        }
        return positions;
    }
    /** Must be called from a subsystem's periodic loop or otherwise consistently run.
     * Odometry tracking & positions will not be correct / reliable otherwise
     */
    @SuppressWarnings("unused")
    public void updateOdometry(){
        odometry.update(gyro.getRotation2d(), getModulePositions());
    }
    /** Stops the swerve drive & sets its target speeds to {0, 0, 0}*/
    @SuppressWarnings("unused")
    public void stop(){
        for (ModuleOrdering ord : ModuleOrdering.values()){
            modules[ord.ordinal()].stop();
        }
    }
}