package frc.robot.subsystems.indexer;

public class IndexerConstants {

	public enum IntakePosition{
		// what constants to put here. 
        // START()     ?

		public final double value;
		private IntakePosition(double value){
			this.value = value;
		}
	}

    public static final TalonFXConfiguration indexerConfig = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(0))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast));

}