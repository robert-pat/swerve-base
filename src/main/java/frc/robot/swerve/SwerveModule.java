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

        driveMotor.config_kP(0, SwerveModuleSetting.DrivePIDConstants[0]);
        driveMotor.config_kI(0, SwerveModuleSetting.DrivePIDConstants[1]);
        driveMotor.config_kD(0, SwerveModuleSetting.DrivePIDConstants[2]);

        steerMotor.config_kP(0, SwerveModuleSetting.SteerPIDConstants[0]);
        steerMotor.config_kI(0, SwerveModuleSetting.SteerPIDConstants[1]);
        steerMotor.config_kD(0, SwerveModuleSetting.SteerPIDConstants[2]);
        steerMotor.setSelectedSensorPosition(
                SwMath.nativeToDegrees(headingEncoder.getAbsolutePosition())
        );
    }
    protected Rotation2d getCurrentAngle(){
        return Rotation2d.fromDegrees(
            SwMath.nativeToDegrees(steerMotor.getSelectedSensorPosition())
        );
    }
    protected void setTargetState(SwerveModuleState state){
        state = SwerveModuleState.optimize(state, getCurrentAngle());
        driveMotor.set(
            ControlMode.Velocity,
            SwMath.MeterPerSecondToNative(state.speedMetersPerSecond)
        );
        // calculate an offset between curr & target headings, add offset to current pos & target it
        steerMotor.set(
            ControlMode.Position,
            SwMath.nativeToDegrees(
                state.angle.minus(getCurrentAngle()).getDegrees() // may need to invert angle diff
            ) + steerMotor.getSelectedSensorPosition()
        );
    }
    protected SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
                SwMath.nativeToMetersPerSecond(driveMotor.getSelectedSensorPosition()),
                getCurrentAngle()
        );
    }
    protected void resetPosition(){
        driveMotor.setSelectedSensorPosition(0);
    }
    protected void stop(){
        setTargetState(new SwerveModuleState());
        driveMotor.neutralOutput();
        steerMotor.neutralOutput();
    }
}
