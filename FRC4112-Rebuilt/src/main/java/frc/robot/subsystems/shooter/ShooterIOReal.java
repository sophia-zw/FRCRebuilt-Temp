package frc.robot.subsystems.shooter;
import static edu.wpi.first.units.Units.Amps; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Amps
import static edu.wpi.first.units.Units.Degrees; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Degrees
import static edu.wpi.first.units.Units.DegreesPerSecond; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#DegreesPerSecond
import static edu.wpi.first.units.Units.Volts; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Volts

import com.ctre.phoenix6.BaseStatusSignal; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/BaseStatusSignal.html
import com.ctre.phoenix6.StatusSignal; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/StatusSignal.html

import com.ctre.phoenix6.controls.MotionMagicVoltage; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/controls/MotionMagicVoltage.html
import com.ctre.phoenix6.controls.VoltageOut; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/controls/VoltageOut.html
import com.ctre.phoenix6.hardware.ParentDevice; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/hardware/ParentDevice.html
import com.ctre.phoenix6.hardware.Pigeon2; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/hardware/Pigeon2.html
import com.ctre.phoenix6.hardware.TalonFX; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/hardware/TalonFX.html

import au.grapplerobotics.LaserCan; //N/A
import edu.wpi.first.math.filter.Debouncer; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/filter/Debouncer.html

import edu.wpi.first.units.measure.Angle; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/Angle.html
import edu.wpi.first.units.measure.AngularVelocity; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/AngularVelocity.html
import edu.wpi.first.units.measure.Current; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/Current.html
import edu.wpi.first.units.measure.Voltage; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/Voltage.html
public class ShooterIOReal implements ShooterIO {
    public ShooterIOReal() {
        
    }
    public void updateInputs(ShooterIOInputs io) {

    }
}
