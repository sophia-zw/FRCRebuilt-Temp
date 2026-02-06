package frc.robot.subsystems.indexer;

public class IndexerConstants {

	public enum IntakePosition{
        START(0); // facing shooter?? this is an assumption. will change. start value to use as reference in case need to get indexer position relative to shooter

		public final double value;
		private IntakePosition(double value){
			this.value = value;
		}
	}
    public static final double indexerVoltage = 0;

    public static final TalonFXConfiguration indexerConfig = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(0))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast));

}