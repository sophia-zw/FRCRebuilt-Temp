package frc.robot.subsystems.indexer;

import frc.robot.subsystems.indexer.IndexerConstants.IndexerPosition;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOReal implements IndexerIO {
    private final TalonFX indexer; 

    private final VoltageOut indexVol = new VoltageOut(0.0);

    private final StatusSignal<AngularVelocity> indexerVel;
    private final StatusSignal<Voltage> indexerAppliedVolts;
    private final StatusSignal<Current> indexerCurrent;

    private final Debouncer indexerConnectedDebounce = new Debouncer(0.5); 

    public IndexerIOReal() {
        indexer = new TalonFX(Ports.INDEXER); 
        indexer.getConfigurator().apply(IntakeConstants.indexerConfig);
        indexerVel = indexer.getVelocity();
        indexerAppliedVolts = indexer.getMotorVoltage();
        indexerCurrent = indexer.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, indexerVel, indexerAppliedVolts, indexerCurrent);
        ParentDevice.optimizeBusUtilizationForAll(indexer);  
    }

    public void updateInputs(IntakeIOInputs inputs) {
        var indexerStatus = BaseStatusSignal.refreshAll(indexerVel, indexerAppliedVolts, indexerCurrent);
        inputs.indexerConnected = indexerConnectedDebounce.calculate(indexerStatus.isOK());
        inputs.indexerVelocityDegPerSec = indexerVel.getValue().in(DegreesPerSecond);
        inputs.indexerAppliedVolts = indexerAppliedVolts.getValue().in(Volts);
        inputs.indexerCurrent = indexerCurrent.getValue().in(Amps);
    }

    @Override 
    public void setIndexer(double output) {
        indexer.setVoltage(output);
    }

    @Override
    public void setIndexerClosedLoop(IndexerPosition pos) {
        indexer.setControl(indexVol.withPosition(Rotations.convertFrom(pos.value, Degrees))); // check this again. could be wrong. 
    }

    @Override
    public void setIndexerOpenLoop(double volts) {
        indexer.setVoltage(volts);
    }

    @Override
    public void resetState() {
        indexer.setPosition(Rotations.convertFrom(IndexerPosition.START.value, Degrees));
    }
}

// ADD IMPORTS