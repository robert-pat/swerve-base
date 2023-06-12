package frc.robot.swerve.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@SuppressWarnings("unused")
class SimDrive {
    private final SimulatedModule[] modules = {
            new SimulatedModule(),
            new SimulatedModule(),
            new SimulatedModule(),
            new SimulatedModule()
    };
    public SwerveModuleState[] getCurrentState(){
        SwerveModuleState[] s = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++){
            s[i] = modules[i].getState();
        }
        return s;
    }
    public Pose2d getFieldPosition() {
        return  new Pose2d(); //TODO:
    }
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return new ChassisSpeeds(); //TODO:
    }
}
