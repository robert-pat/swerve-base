package frc.robot.simulation;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveConstants;
import frc.robot.swerve.SwerveDisplay;
import frc.robot.swerve.SwerveDrive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SimulationRunner extends SubsystemBase {
    DoubleSupplier[] axis;
    boolean isAxisRegistered = false;
    final SwerveDrive drive = new SwerveDrive();
    final SwerveDisplay display = new SwerveDisplay(drive);
    final Supplier<DriveMode> modeController;
    final BooleanPublisher activityDisplay = NetworkTableInstance.getDefault()
                                                            .getTable("swerve-data")
                                                            .getBooleanTopic("isActive")
                                                            .publish();
    public SimulationRunner(Supplier<DriveMode> supplier){
        this.modeController = supplier;
        if (RobotBase.isReal()) {
            DriverStation.reportError(
                    "Swerve Simulation Runner Constructed, but robot is not simulated!",
                    false
            );
        }
    }
    @SuppressWarnings("unused")
    public SimulationRunner(){
        this(()-> DriveMode.VanceDrive);
    }
    public void registerAxis(DoubleSupplier lx, DoubleSupplier ly, DoubleSupplier rx, DoubleSupplier ry){
        axis = new DoubleSupplier[]{lx, ly, rx, ry};
        isAxisRegistered = true;
    }
    public void updateDriveTarget(){
        //noinspection SwitchStatementWithTooFewBranches
        switch (modeController.get()){
            case VanceDrive:
                drive.driveSwerve(new ChassisSpeeds(
                        axis[0].getAsDouble() * SwerveConstants.MaxModuleMetersPerSecond,
                        axis[1].getAsDouble() * SwerveConstants.MaxModuleMetersPerSecond,
                        axis[2].getAsDouble() * SwerveConstants.MaxRadiansPerSecond
                ), false);
            break;

            default:
                DriverStation.reportWarning("No DriveMode Selected", false);
            break;
        }
    }
    @Override
    public void simulationPeriodic() {
        activityDisplay.set(true);
        if(isAxisRegistered){
            updateDriveTarget();
        }
        display.updateDisplay();
    }
}
