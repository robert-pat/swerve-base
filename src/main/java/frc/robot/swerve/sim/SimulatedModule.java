package frc.robot.swerve.sim;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SimulatedModule {
    private SwerveModuleState currentState = new SwerveModuleState();
    private SwerveModulePosition currentPosition = new SwerveModulePosition();

    protected void updateState(){
        updateState(currentState);
    }
    protected void updateState(SwerveModuleState newState){
        currentState = newState;
        currentPosition = new SwerveModulePosition(
                currentPosition.distanceMeters + (newState.speedMetersPerSecond * 0.02),
                newState.angle
        );
    }
    protected SwerveModuleState getState(){
        return currentState;
    }
    protected SwerveModulePosition getPosition(){
        return currentPosition;
    }
}
