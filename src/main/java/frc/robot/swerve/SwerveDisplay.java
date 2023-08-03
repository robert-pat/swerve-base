package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SwerveDisplay {
    final static int DisplayWidth = 50;
    final static int DisplayHeight = 50;
    final static int DriveBaseCornerLength = 15;
    final static int MaxWheelLength = 8;
    final static int MinWheelLength = 1;
    private final SwerveDrive drive;
    @SuppressWarnings("resource")
    private final MechanismLigament2d[] offsets = {
            new MechanismLigament2d("FrontLeftOffset", DriveBaseCornerLength, 135, 5, new Color8Bit(Color.kWhite)),
            new MechanismLigament2d("BackLeftOffset", DriveBaseCornerLength, 225, 5, new Color8Bit(Color.kWhite)),
            new MechanismLigament2d("FrontRightOffset", DriveBaseCornerLength, 45, 5, new Color8Bit(Color.kWhite)),
            new MechanismLigament2d("BackRightOffset", DriveBaseCornerLength, 315, 5, new Color8Bit(Color.kWhite))
    };
    final DoublePublisher odometryHeadingPub;
    final DoubleArrayPublisher odometryPositionPub;
    final DoubleArrayPublisher kinematicsVelPublisher;
    @SuppressWarnings("resource")
    private final MechanismLigament2d[] wheels = {
            new MechanismLigament2d("FrontLeftWheel", MinWheelLength, 90, 5, new Color8Bit(Color.kRed)),
            new MechanismLigament2d("BackLeftWheel", MinWheelLength, 90, 5, new Color8Bit(Color.kRed)),
            new MechanismLigament2d("FrontRightWheel", MinWheelLength, 90, 5, new Color8Bit(Color.kRed)),
            new MechanismLigament2d("BackRightWheel", MinWheelLength, 90, 5, new Color8Bit(Color.kRed))
    };
    public SwerveDisplay(SwerveDrive drive){
        this.drive = drive;
        Mechanism2d mechanism = new Mechanism2d(
                DisplayWidth,
                DisplayHeight,
                new Color8Bit(0, 0, 0)
        );
        MechanismRoot2d rootNode = mechanism.getRoot("center", (double) DisplayWidth / 2, (double) DisplayHeight / 2);


        for (MechanismLigament2d l : offsets){
            rootNode.append(l);
        }
        for (ModuleOrdering ord : ModuleOrdering.values()){
            offsets[ord.ordinal()].append(wheels[ord.ordinal()]);
        }
        SmartDashboard.putData("SwerveDisplay", mechanism);

        NetworkTable t = NetworkTableInstance.getDefault().getTable("swerve-data");
        odometryHeadingPub = t.getDoubleTopic("chassis_heading").publish();
        odometryPositionPub = t.getDoubleArrayTopic("chassis_position").publish();
        kinematicsVelPublisher = t.getDoubleArrayTopic("chassis_velocity").publish();
    }

    /** Currently doesn't display anything b/c PID constants are 0 and/or Falcons don't simulate
     * their closed loop outputs in the simulator. The display code is fine. (tho wheel angles are untested) */
    public void updateDisplay(){
        SwerveModuleState[] states = drive.getCurrentState();
        for (ModuleOrdering ord : ModuleOrdering.values()){
            MechanismLigament2d l = wheels[ord.ordinal()];
            SwerveModuleState s = states[ord.ordinal()];

            Rotation2d angle = s.angle.minus(
                    Rotation2d.fromDegrees(offsets[ord.ordinal()].getAngle())
            );
            l.setAngle(angle);
            double len = (s.speedMetersPerSecond / SwerveConstants.MaxModuleMetersPerSecond) * MaxWheelLength;
            l.setLength(Math.max(len, 0.75));
            if (len == 0){
                l.setColor(new Color8Bit(Color.kBlue));
            }
            else{
                l.setColor(new Color8Bit(Color.kRed));
            }
        }

        Pose2d pose = drive.getFieldPosition();
        odometryHeadingPub.set(pose.getRotation().getDegrees());
        odometryPositionPub.set(new double[]{pose.getX(), pose.getY()});

        ChassisSpeeds speeds = drive.getRobotRelativeSpeeds();
        kinematicsVelPublisher.set(new double[]{
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond
        });
    }
}
