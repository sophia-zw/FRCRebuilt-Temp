package frc.robot.subsystems.shooter; 

import org.littletonrobotics.junction.AutoLog; 

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        //turretCur, anglerCur, fuelCur, turretVol, fuelVol, anglerVol, fuelVel, turretVel, anglerVel, anglerPos

        public boolean turretConnected = false;
        public boolean fuelConnected = false;
        public boolean fuel2Connected = false;
        public boolean anglerConnected = false;
        public boolean wheelsConnected = false;

        public double anglerCur = 0;
        public double fuelCur = 0;
        public double turretCur = 0;
        public double wheelsCur = 0;

        public double anglerVol = 0;
        public double fuelVol = 0;
        public double turretVol = 0;
        public double wheelsVol = 0;

        public double anglerVel = 0;
        public double fuelVel = 0;
        public double turretVel = 0;
        public double wheelsVel = 0;

        public double anglerPos = 0;

        

        

    }
    public default void updateInputs(ShooterIOInputs io){}
    
    public default void setTurretClosedLoop(double value){}

    public default void setTurretOpenLoop(double value){}

    public default void setAnglerClosedLoop(double value){}

    public default void setAnglerOpenLoop(double value){}

    public default void setWheelsVoltage(double value){}

    public default void setFuelVoltage(double value){}

    public default void resetState(double value){}

    public default void syncPigeon(){}
}
