package frc.robot.subsystems.climb; //N/A

import org.littletonrobotics.junction.AutoLog; //N/A
//THESE ALL NEED TO BE CHANGED***************************************************************************************************888
public interface ClimbIO {
    @AutoLog
    //The class variables will be used in the method update inputs
    public static class ClimbIOInputs {
        public boolean climbConnected = false; //Checks if it is connected
        public double climbPositionDeg = 0; //What is the climb bosition in degrees
        public double climbVelocityDegPerSec = 0; //What is the climb velocity in degrees per second
        public double climbVoltage = 0; //What is the climb voltage
        public double climbCurrent = 0; //What is the climb current
    }

    public default void updateInputs(ClimbIOInputs inputs) {} //Updates climb inputs

    public default void setClimbClosedLoop(double pos) {} //Sets the climb closed loop to a certain position

    public default void resetState() {} //Resets the climb to its normal configuration

    public default void stopClimb() {} //Stops the climb 

}