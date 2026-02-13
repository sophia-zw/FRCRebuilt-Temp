package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog; 

import frc.robot.subsystems.indexer.IndexerConstants.IndexerPosition;

public interface IndexerIO {
    public boolean indexerConnected = false;
    public double indexerVelocityDegPerSec = 0;
    public double indexerAppliedVolts = 0;
    public double indexerCurrent = 0;

    @AutoLog
    public static class IndexerIOInputs {

        public default void updateInputs(IndexerIOInputs inputs) {}

        public default void setIndexer(double output) {}

        public default void setIndexerClosedLoop(IndexerPosition pos) {} // ????

        public default void setIndexerOpenLoop(double volts) {} // ???
        
        public default void resetState();
    }
}


// later thoughts
    // dependig on + / - velocity, can also determine direction of movement of indexer..
    // but its only spinning in one direction if its working with shooter during comp. 
    // so need to get position? to get relative position, need to set something on encoder to use as relative point..

//ADD IMPORTS