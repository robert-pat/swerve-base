package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.geometry.Translation2d;
import jdk.jfr.Description;

public class SwerveModuleSetting {
    final int DriveCANID;
    final int SteerCANID;
    final int EncoderCANID;
    final TalonFXConfiguration DriveConfig = new TalonFXConfiguration();
    final TalonFXConfiguration SteerConfig = new TalonFXConfiguration();
    final CANCoderConfiguration EncoderConfig = new CANCoderConfiguration();
    final ModuleOrdering Ordering;
    public final Translation2d Position;
    public static final double[] DrivePIDConstants = {0, 0, 0};
    public static final double[] SteerPIDConstants = {0, 0, 0};

    /** Holds all the settings objects (and specific values) for configuring a swerve module
     * @param dID drive motor CAN ID
     * @param sID steer motor CAN ID
     * @param eID encoder CAN ID
     * @param encOffset the offset, in degrees, for the modules absolute encoder
     * @param ord which module the settings are for
     * @param p where the module is located
     * */
    public SwerveModuleSetting(int dID, int sID, int eID, double encOffset, ModuleOrdering ord, Translation2d p){
        DriveCANID = dID;
        SteerCANID = sID;
        EncoderCANID = eID;

        Ordering = ord;

        EncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        EncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        EncoderConfig.magnetOffsetDegrees = encOffset;

        Position = p;
    }
}

/**The logical location / ordering of the swerve modules. <br>
 * If using the .values() / .ordinal() methods, array entries <b>MUST</b> be
 * created in the same order as the enum variants are defined here: */
enum ModuleOrdering {
    FrontLeft,
    BackLeft,
    FrontRight,
    BackRight
}

