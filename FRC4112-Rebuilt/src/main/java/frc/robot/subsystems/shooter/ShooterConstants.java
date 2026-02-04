package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Degrees
import static edu.wpi.first.units.Units.Radians; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Radians

import com.ctre.phoenix6.configs.CurrentLimitsConfigs; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/CurrentLimitsConfigs.html
import com.ctre.phoenix6.configs.FeedbackConfigs; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/FeedbackConfigs.html
import com.ctre.phoenix6.configs.MotionMagicConfigs; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/MotionMagicConfigs.html
import com.ctre.phoenix6.configs.MotorOutputConfigs; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html
import com.ctre.phoenix6.configs.MountPoseConfigs; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/MountPoseConfigs.html
import com.ctre.phoenix6.configs.Pigeon2Configuration; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/Pigeon2Configuration.html
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/Pigeon2FeaturesConfigs.html
import com.ctre.phoenix6.configs.Slot0Configs; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/Slot0Configs.html
import com.ctre.phoenix6.configs.TalonFXConfiguration; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/TalonFXConfiguration.html
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/signals/FeedbackSensorSourceValue.html
import com.ctre.phoenix6.signals.GravityTypeValue; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/signals/GravityTypeValue.html
import com.ctre.phoenix6.signals.InvertedValue; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/signals/InvertedValue.html
import com.ctre.phoenix6.signals.NeutralModeValue; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/signals/NeutralModeValue.html
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/signals/StaticFeedforwardSignValue.html
public class ShooterConstants {
	/*
	public enum ShooterPosition {
		START(0);
	

	
		
		public final double value;
		private ShooterPosition(double value){
			this.value = value;
		}
	}
	*/

	public enum TurretPosition {
		START(0);
	

	
		
		public final double value;
		private TurretPosition(double value){
			this.value = value;
		}
	}

	public enum AnglerPosition {
		START(0);
	

	
		
		public final double value;
		private AnglerPosition(double value){
			this.value = value;
		}
	}

	public static final double fuelVoltage = 0;
	public static final double fuelHoldVoltage = 0;

	/*didn't do angler yet, cause that part is not clear
	may need a pigeon config too
	
	The stator vs supply needs to be differientiated

	Motor configs are REAL iFFY
	*/

	public static final TalonFXConfiguration turretConfig = new TalonFXConfiguration()
		.withCurrentLimit(new CurrentLimitsConfigs()
			.withSupplyCurrentLimitEnable(true) //This might be .withStatorCurrentLimitEnable but seeing as its not a pivot action I did Supply
			.withSupplyCurrentLimit(20)
		)
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.Clockwise_Positive) //IDK if this is cw or ccw. I just put cw for the time being
			.withNeutralMode(NeutralMode.Brake)
		);

	public static final TalonFXConfiguration fuelConfig = new TalonFXConfiguration()
		.withCurrentLimit(new CurrentLimitsConfigs()
			.withSupplyCurrentLimitEnable(true) //This might be .withStatorCurrentLimitEnable but seeing as its not a pivot action I did Supply
			.withSupplyCurrentLimit(20)
		)
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.Clockwise_Positive) //IDK if this is cw or ccw. I just put cw for the time being
			.withNeutralMode(NeutralMode.Brake)
		);
	
}