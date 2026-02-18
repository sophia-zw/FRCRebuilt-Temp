package frc.robot.subsystems.elevator;  //N/A

import static edu.wpi.first.units.Units.Amps; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Amps
import static edu.wpi.first.units.Units.Volts; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Volts

import com.ctre.phoenix6.BaseStatusSignal; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/BaseStatusSignal.html
import com.ctre.phoenix6.StatusSignal; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/StatusSignal.html
import com.ctre.phoenix6.controls.MotionMagicVoltage; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/controls/MotionMagicVoltage.html
import com.ctre.phoenix6.hardware.ParentDevice; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/hardware/ParentDevice.html
import com.ctre.phoenix6.hardware.TalonFX;//https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/hardware/TalonFX.html

import edu.wpi.first.units.measure.Angle;//https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/Angle.html
import edu.wpi.first.units.measure.AngularVelocity; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/AngularVelocity.html
import edu.wpi.first.units.measure.Current; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/Current.html
import edu.wpi.first.units.measure.Voltage; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/Voltage.html
import frc.robot.Constants.Ports; //N/A
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPosition; //N/A
//THESE ALL NEED TO BE CHANGED***************************************************************************************************888
public class ElevatorIOReal implements ElevatorIO{
    private final TalonFX elevMotor;
    private MotionMagicVoltage position = new MotionMagicVoltage(0);

    private StatusSignal<Voltage> elevVol;
    private StatusSignal<Angle> elevPos;
    private StatusSignal<AngularVelocity> elevVel;
    private StatusSignal<Current> elevCur;

    public ElevatorIOReal(){
        elevMotor = new TalonFX(Ports.ELEVATOR_BOTTOM);
        
        elevVol = elevMotor.getMotorVoltage();
        elevPos = elevMotor.getPosition(); // Is actually inches
        elevVel = elevMotor.getVelocity(); // Is actually inches/sec
        elevCur = elevMotor.getStatorCurrent();

        elevMotor.getConfigurator().apply(ElevatorConstants.elevConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(50, elevVol, elevPos, elevVel, elevCur); 
        ParentDevice.optimizeBusUtilizationForAll(elevMotor);
        
    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        var elevStatus = BaseStatusSignal.refreshAll(elevVol, elevPos, elevVel, elevCur);

        inputs.elevConnected = elevStatus.isOK();
        inputs.elevVelocityInchesPerSec = elevVel.getValueAsDouble();
        inputs.elevPositionInches = elevPos.getValueAsDouble();
        inputs.elevVoltage = elevVol.getValue().in(Volts);
        inputs.elevCurrent = elevCur.getValue().in(Amps);
    }

    @Override
    public void setElevatorClosedLoop(double pos){ 
        elevMotor.setControl(position.withPosition(pos)); 
    }

    @Override
    public void setElevatorOpenLoop(double volts){
        elevMotor.setVoltage(volts);
    }

    @Override
    public void resetState() {
        elevMotor.setPosition(ElevatorPosition.START.value); 
    }

    @Override
    public void stopElevator(){
        elevMotor.stopMotor();
    }
}