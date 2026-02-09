package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
//ROMAN needs to do these constants
public class IntakeConstants {
	public enum IntakePosition {
        LOWERED(0),
        STRAIGHT(0),
        RETRACTED(0),
        START(0),
        AVOID(0);

        public final double value;

        private IntakePosition(double value) {
            this.value = value;
        }
    }

    public static final double wheelVoltage = 0;
    public static final double indexerVoltage = 0;
    public static final double pivotTolerance = 0;
    public static final double minSensorDistance = 0;
    public static final double stuckFuelTimeSeconds = 0;

    public static final TalonFXConfiguration pivotConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(0))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(0))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(0)
                    .withMotionMagicCruiseVelocity(0))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs()
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                    .withKS(0)
                    .withKV(0)
                    .withKA(0)
                    .withKG(0)
                    .withKP(0)
                    .withKI(0.0)
                    .withKD(0.0));

    public static final TalonFXConfiguration wheelConfig = new TalonFXConfiguration()
    .withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(0))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast));
}