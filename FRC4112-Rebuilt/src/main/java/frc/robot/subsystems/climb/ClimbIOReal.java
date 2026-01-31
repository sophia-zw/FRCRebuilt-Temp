package frc.robot.subsystems.climb; //N/A

import static edu.wpi.first.units.Units.Amps; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Amps
import static edu.wpi.first.units.Units.Degrees; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Degrees
import static edu.wpi.first.units.Units.DegreesPerSecond; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#DegreesPerSecond
import static edu.wpi.first.units.Units.Rotations; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Rotations
import static edu.wpi.first.units.Units.Volts; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Volts

import com.ctre.phoenix6.BaseStatusSignal; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/BaseStatusSignal.html
import com.ctre.phoenix6.StatusSignal; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/StatusSignal.html
import com.ctre.phoenix6.controls.PositionVoltage; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/controls/PositionVoltage.html
import com.ctre.phoenix6.hardware.ParentDevice; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/hardware/ParentDevice.html
import com.ctre.phoenix6.hardware.TalonFX; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/hardware/TalonFX.html


import edu.wpi.first.units.measure.Angle; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/Angle.html
import edu.wpi.first.units.measure.AngularVelocity; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/AngularVelocity.html
import edu.wpi.first.units.measure.Current; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/Current.html
import edu.wpi.first.units.measure.Voltage; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/Voltage.html
import frc.robot.Constants.Ports; //N/A
import frc.robot.subsystems.climb.ClimbConstants.ClimbPosition; //N/A
//THESE ALL NEED TO BE CHANGED***************************************************************************************************888
public class ClimbIOReal implements ClimbIO{
    private final TalonFX climbMotor; //Intitalizes the climbMotor to be from TalonFX

    private final PositionVoltage position = new PositionVoltage(0); //Request PID to target position with voltage feedforward

    /*StatusSignal<t> is a class which has a data type inside <>. Its used to recieve different informations about the signal. You can think of it as a wrapper that holds data.
    It has prebuilt methods that allow us to retrieve method about the data.
    */
    
    private StatusSignal<Voltage> climbVol; //Holds voltage for the motor of the climb.
    private StatusSignal<Angle> climbPos; // Holds the angle for the position for the clim
    private StatusSignal<AngularVelocity> climbVel; // Holds the angular velocity for the climb
    private StatusSignal<Current> climbCur; //Holds the currecnt for the climb. 

    public ClimbIOReal(){
        climbMotor = new TalonFX(Ports.CLIMB); //Intializes the climb motor object
            /*The status signals are updated from the motor controller.  These refreshes and returns a cached StatusSignal object.
            All the methods shown below are prebuilt with the purpose of refreshing and retutning values for all the Status signal objects.
            */ 

        climbVol = climbMotor.getMotorVoltage(); //Sets the output of the motor voltage using a prebuilt method from the TalonFX class. 
        climbPos = climbMotor.getPosition();      //Sets the position by using the prebuilt method from the TalonFX class. It sets the position in rotations.
        climbVel = climbMotor.getVelocity();    //Sets the velocity by using the prebuilt velocity from the TalonFX class. Its sets the velocity in rotations per second. 
        climbCur = climbMotor.getStatorCurrent();    /*Stator current where Positive current indicates motoring regardless of direction. 
    Negative current indicates regenerative braking regardless of direction. A more simplified explanation is that if it is positive it shows how 
    its moving in distance regardless of front or back. Negative means that its slowing down regardless if its moving forwards or backwards.

            */

        BaseStatusSignal.setUpdateFrequencyForAll(50, climbVol, climbPos, climbVel, climbCur); /* It will set all the status signals in a specific frequency that is a double.
            In this particular example we can see how we are setting the frequency for all the values to the double value that was listed of 50 herz. It's important to note
            that if the signal is 0 it will stop, the minimum signal is 4, and the maximum is 1000
            
            */
        
        ParentDevice.optimizeBusUtilizationForAll(climbMotor); /* This will optimise the bus utilization. Basically the Controller Area Network (Can) bus bandwidth 
        refers to the amount of data that can be send per second. This method allows us to reduce update frequencies for that particular status signal.

            */

        climbMotor.getConfigurator().apply(ClimbConstants.climbConfig);  //Gets the configurator to use with this device's configs. This method returns a configurator for the object that has been used for the method.
      
    }

    @Override
    //The method below will update inputs and it uses a object from CLIMBIOInputs that has been passed down.
    public void updateInputs(ClimbIOInputs inputs) {

        BaseStatusSignal.refreshAll(climbVol, climbPos, climbVel, climbCur); //Refreshes all the status signals so you dont have to do them individually.
        inputs.climbVelocityDegPerSec = climbVel.getValue().in(DegreesPerSecond); // Setting the climb velocity from rotations per second to degrees per second
        inputs.climbVoltage = climbVol.getValue().in(Volts); //Setting the climb volatge in volts
        inputs.climbPositionDeg = climbPos.getValue().in(Degrees); //Setting the climb position in degrees.
        inputs.climbCurrent = climbCur.getValue().in(Amps); //Setting the climb current in amps
    }

    @Override
    public void setClimbClosedLoop(double pos) {
        climbMotor.setControl(position.withPosition(pos));  //Sets the position with the parameter that has been passed down
    }

    @Override
    public void resetState(){
        climbMotor.setPosition(Rotations.convertFrom(ClimbPosition.START.value, Degrees)); //Sets the postition to its basic configuration
    } 

    @Override
    public void stopClimb(){
        climbMotor.stopMotor(); //Stops the motor of the Climb until its called again
    }
}
    