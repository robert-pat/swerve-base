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
 * 1) init() <br>
 * 2) updateOdometry()
 */
@SuppressWarnings("unused")
public class SwerveDrive {
    private final SwerveModule[] modules = {
            new SwerveModule(SwerveConstants.ModuleSettings.FrontLeft),
            new SwerveModule(SwerveConstants.ModuleSettings.BackLeft),
            new SwerveModule(SwerveConstants.ModuleSettings.FrontRight),
            new SwerveModule(SwerveConstants.ModuleSettings.BackRight)
    };
    protected final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            FrontLeft.Position,
            BackLeft.Position,
            FrontRight.Position,
            BackRight.Position
    );
    protected final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            kinematics,
            new Rotation2d(),
            new SwerveModulePosition[]{
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            }
    );
    public final AHRS gyro = new AHRS();

    /*External Control Methods*/

    /**Initializes odometry & performs initial setup for each swerve module. <br>
     * This method <b>MUST</b> be called once the robot has started up to properly
     * initialize the swerve modules.*/
    @SuppressWarnings("unused")
    public void init(){
        odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), new Pose2d());
        for (SwerveModule s : modules){
            s.init();
        }
    }
    /**Commands the robot to drive
     * @param speeds the desired speeds
     * @param isFieldRelative whether the speeds are relative to the field
     */
    @SuppressWarnings("unused")
    public void driveSwerve(ChassisSpeeds speeds, boolean isFieldRelative){
        if (isFieldRelative){
            speeds = adjustFieldRelativeSpeeds(speeds);
        }
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        applyUnadjustedModuleStates(states);
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
            speeds = adjustFieldRelativeSpeeds(speeds);
        }
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds, center);
        applyUnadjustedModuleStates(states);
    }
    public void driveSwerveOpenLoop(ChassisSpeeds speeds, boolean isFieldRelative){
        if (isFieldRelative){
            speeds = adjustFieldRelativeSpeeds(speeds);
        }
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MaxModuleMetersPerSecond);
        for (ModuleOrdering ord : ModuleOrdering.values()){
            SwerveModuleState s = states[ord.ordinal()];
            modules[ord.ordinal()].setOpenLoop(s.angle, s.speedMetersPerSecond / MaxModuleMetersPerSecond);
        }
    }

    /*Internal control methods*/
    protected void applyUnadjustedModuleStates(SwerveModuleState[] states){
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MaxModuleMetersPerSecond);
        for (ModuleOrdering ord : ModuleOrdering.values()){
            modules[ord.ordinal()].setTargetState(states[ord.ordinal()]);
        }
    }
    /**Converts the fieldRelative speeds to be robot relative
     * @param fieldRelative the field relative speeds
     * @return robot relative speeds
     */
    protected ChassisSpeeds adjustFieldRelativeSpeeds(ChassisSpeeds fieldRelative){
        Translation2d v = new Translation2d(fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond);
        v = v.rotateBy(gyro.getRotation2d().unaryMinus());
        return new ChassisSpeeds(
                v.getX(),
                v.getY(),
                fieldRelative.omegaRadiansPerSecond
        );
    }

    /*Odometry & Position Tracking Methods*/
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (ModuleOrdering ord : ModuleOrdering.values()){
            positions[ord.ordinal()] = modules[ord.ordinal()].getPosition();
        }
        return positions;
    }
    public SwerveModuleState[] getCurrentState(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (ModuleOrdering ord : ModuleOrdering.values()){
            states[ord.ordinal()] = modules[ord.ordinal()].getState();
        }
        return states;
    }
    /** Must be called from a subsystem's periodic loop or otherwise consistently run.
     * Odometry tracking & positions will not be correct / reliable otherwise
     */
    @SuppressWarnings("unused")
    public void updateOdometry(){
        odometry.update(gyro.getRotation2d(), getModulePositions());
    }
    @SuppressWarnings("unused")
    public void resetAllOdometry(Pose2d fieldPos){
        odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), fieldPos);
    }
    public Pose2d getFieldPosition(){
        return odometry.getPoseMeters();
    }
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return kinematics.toChassisSpeeds(getCurrentState());
    }
    /** Stops the swerve drive & sets its target speeds to {0, 0, 0}*/
    @SuppressWarnings("unused")
    public void stop(){
        for (ModuleOrdering ord : ModuleOrdering.values()){
            modules[ord.ordinal()].stop();
        }
    }
}