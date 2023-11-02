package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    final SwerveModuleSetting settings;
    final WPI_TalonFX driveMotor;
    final WPI_TalonFX steerMotor;
    final WPI_CANCoder headingEncoder;
    public SwerveModule(SwerveModuleSetting settings){
        this.settings = settings;

        driveMotor = new WPI_TalonFX(settings.DriveCANID);
        steerMotor = new WPI_TalonFX(settings.SteerCANID);

        headingEncoder = new WPI_CANCoder(settings.EncoderCANID);
    }

    public void init(){
        driveMotor.configAllSettings(settings.DriveConfig);
        steerMotor.configAllSettings(settings.SteerConfig);
        headingEncoder.configAllSettings(settings.EncoderConfig);

        //TODO: CAN status frame configurations

        driveMotor.config_kP(0, SwerveConstants.DrivePIDConstants[0]);
        driveMotor.config_kI(0, SwerveConstants.DrivePIDConstants[1]);
        driveMotor.config_kD(0, SwerveConstants.DrivePIDConstants[2]);

        steerMotor.config_kP(0, SwerveConstants.SteerPIDConstants[0]);
        steerMotor.config_kI(0, SwerveConstants.SteerPIDConstants[1]);
        steerMotor.config_kD(0, SwerveConstants.SteerPIDConstants[2]);
        steerMotor.setSelectedSensorPosition(
                // this might need to be degreesToNative() -future robert (wrote this months ago)
                SwerveConstants.degreesToNative(headingEncoder.getAbsolutePosition())
        );
    }
    protected Rotation2d getCurrentAngle(){
        return Rotation2d.fromDegrees(
            SwerveConstants.nativeToDegrees(steerMotor.getSelectedSensorPosition())
        );
    }
    protected void setTargetState(SwerveModuleState state){
        state = SwerveModuleState.optimize(state, getCurrentAngle());
        driveMotor.set(
            ControlMode.Velocity,
            SwerveConstants.MeterPerSecondToNative(state.speedMetersPerSecond)
        );
        // Calculate an offset between curr & target headings, add offset to current pos & target it
        // The offset is guaranteed to be mod 360 because of WPIlib's internal matrix math & the .minus() method
        steerMotor.set(
            ControlMode.Position,
            SwerveConstants.degreesToNative(
                state.angle.minus(getCurrentAngle()).getDegrees() // may need to invert angle diff
            ) + steerMotor.getSelectedSensorPosition()
        );
    }
    protected SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
                SwerveConstants.nativeToMeters(driveMotor.getSelectedSensorPosition()),
                getCurrentAngle()
        );
    }
    protected SwerveModuleState getState(){
        return new SwerveModuleState(
                SwerveConstants.nativeToMetersPerSecond(driveMotor.getSelectedSensorVelocity()),
                getCurrentAngle()
        );
    }
    @SuppressWarnings("unused")
    protected void resetPosition(){
        driveMotor.setSelectedSensorPosition(0);
    }
    protected void stop(){
        // Leaving this as a [0.0 m/s, 0 degree] state is ok b/c the neutralOutput() call
        // means the modules won't actually go to 0 degrees, they'll stay neutral until given a new state
        setTargetState(new SwerveModuleState());
        driveMotor.neutralOutput();
        steerMotor.neutralOutput();
    }

    /**Runs the module in "open loop" mode, w/o a PID controller on the drive motor. <br>
     * Note that heading control of any kind requires a tuned PID controller for the steer motor.
     * @param heading the heading the module wheel should face
     * @param dPower how much power for the drive motor, on  the interval [-1,1]
     */
    protected void setOpenLoop(Rotation2d heading, double dPower){
        driveMotor.set(dPower);
        steerMotor.set(
            ControlMode.Position,
            SwerveConstants.degreesToNative(heading.minus(getCurrentAngle()).getDegrees()) + steerMotor.getSelectedSensorPosition()
        );
    }

    /**Runs the module in the simplest possible mode, with direct output called to both motors
     * @param dPower the drive power, in & output
     * @param sPower the steer power, in % output
     */
    @SuppressWarnings("unused")
    @Deprecated
    protected void setRaw(double dPower, double sPower){
        driveMotor.set(dPower);
        steerMotor.set(sPower);
    }
}
