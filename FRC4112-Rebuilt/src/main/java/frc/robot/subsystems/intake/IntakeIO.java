package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog; 

import frc.robot.subsystems.intake.IntakeConstants.IntakePosition;
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean pivotConnected;
        public double pivotPositionDeg = 0;
        public double pivotVelocityDegPerSec = 0;
        public double pivotAppliedVolts = 0;;
        public double pivotCurrent = 0;

        public boolean wheelsConnected = false;
        public double wheelsVelocityDegPerSec = 0;
        public double wheelsAppliedVolts = 0;
        public double wheelsCurrent = 0;
        //INDEXER stuff
        public boolean indexerConnected = false;
        public double indexerVelocityDegPerSec = 0;
        public double indexerAppliedVolts = 0;
        public double indexerCurrent = 0;
        
        //SENSOR Stuff 
        public double sensorDistanceMillimeters = 0;
        public boolean laserCANConnected = false;
        public int laserCANStatus = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setPivotClosedLoop(IntakePosition pos) {}

    public default void setPivotOpenLoop(double volts) {}
    
    public default void resetState() {}

    public default void setWheels(double output) {}
//INDEXER STUFF
    public default void setIndexer(double output) {}

    
}