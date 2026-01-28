package frc.robot.subsystems.climb; //N/A

import com.ctre.phoenix6.configs.CurrentLimitsConfigs; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/CurrentLimitsConfigs.html
import com.ctre.phoenix6.configs.FeedbackConfigs; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/FeedbackConfigs.html
import com.ctre.phoenix6.configs.MotorOutputConfigs; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html
import com.ctre.phoenix6.configs.Slot0Configs; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/Slot0Configs.html
import com.ctre.phoenix6.configs.TalonFXConfiguration; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/TalonFXConfiguration.html
import com.ctre.phoenix6.signals.InvertedValue; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/signals/InvertedValue.html
import com.ctre.phoenix6.signals.NeutralModeValue; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/signals/NeutralModeValue.html

public class ClimbConstants {
	public enum ClimbPosition {
		//Enums represent constants that are unchangeable(final)
		START(90.0),
		GRAB(0.0),
		CLIMB(90.0);

		public final double value;  

		private ClimbPosition(double value) {
			this.value = value;     //Sets value as the climb postion value that has been passed down as a parameter
		}
	}

	public static final double climbVoltage = 12.0; //Sets the climb voltage to 12
	public static final double climbMoveVoltage = 5.0; //Sets the voltage for the climb to move at 5
	public static final double climbTolerance = 1; //Sets the tolerance to 1
		//In this code they will work on configuring the talonx fx motor
	public static final TalonFXConfiguration climbConfig = new TalonFXConfiguration()
				//The .with creates new objects that is copied and changes the fields. It allows for easier method chaining
			.withCurrentLimits(new CurrentLimitsConfigs() //Allows you to make limits on the current of the motor. It lets us utilize the code below
					.withSupplyCurrentLimitEnable(true)//Allows for a limit on the current
					.withSupplyCurrentLimit(39)) /*
					Supply current is the absolute maximum amount of supply current allowed. In this example it is set to 39.
     					Supply current comes from the battery so its important to set a limit.
			*/
			.withFeedback(new FeedbackConfigs()//New object for the feedback configuration of the motor
					.withSensorToMechanismRatio(400)) //The ratio of sensor rotations to the mechanism's output, where a ratio greater than 1 is a reduction.
			.withSlot0(new Slot0Configs() //Sets the "gains"(PID) of the robot to slot 0
					.withKP(300))//Sets porportion gain to 300
			.withMotorOutput(new MotorOutputConfigs( //Used to set the configuration of the motor output
					.withInverted(InvertedValue.CounterClockwise_Positive) /*Inverts the state of the device. CounterClockwise_Positive 
     					Positive motor output results in counter-clockwise motion. It can be used nvert the output of the motor. 

						*/
					.withNeutralMode(NeutralModeValue.Brake));  /*Used for he state of the motor controller bridge when output is neutral or disabled.
						NeutralModeValue is a enum that cotains Brake. 
				*/
}