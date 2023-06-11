package frc.robot.swerve;

public class SwMath {
    final static double TicksToDegrees = 1.05;
    final static double TicksToMSP = 1.05;
    static double nativeToDegrees(double n){
        return n *  TicksToDegrees;
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
}