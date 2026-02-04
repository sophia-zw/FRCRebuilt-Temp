package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog; 

import frc.robot.subsystems.indexer.IndexerConstants.IndexerPosition;

public interface IndexerIO {

    //INDEXER stuff
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
    }
}

//ADD IMPORTS