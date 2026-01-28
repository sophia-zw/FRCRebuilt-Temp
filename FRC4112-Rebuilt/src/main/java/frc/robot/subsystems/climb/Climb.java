package frc.robot.subsystems.climb; //N/A

import static edu.wpi.first.units.Units.Degrees; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Degrees
import static edu.wpi.first.units.Units.Rotations; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Rotations

import org.littletonrobotics.junction.AutoLogOutput; //N/A
import org.littletonrobotics.junction.Logger; //N/A

import edu.wpi.first.wpilibj.Alert; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/Alert.html
import edu.wpi.first.wpilibj.Alert.AlertType;  //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/Alert.AlertType.html
import edu.wpi.first.wpilibj2.command.SubsystemBase;  //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SubsystemBase.html
import frc.robot.subsystems.climb.ClimbConstants.ClimbPosition; //N/A

public class Climb extends SubsystemBase {
    private final ClimbIO climbIO;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    private final Alert climbDisconnectedAlert;

    private ClimbPosition targetPosition;   //Declaring all the class variables

    public Climb(ClimbIO io) {
        climbIO = io; //Sets climbIO equal to the parameter that is passed down when the object is created for the Climb.java
        climbDisconnectedAlert = new Alert("Disconected climb motor", AlertType.kError); //Initializes the object from the Alert Class
    }

    @Override
    //The periodic method is called every 20 miliseconds
    public void periodic() {
        climbIO.updateInputs(inputs); //Uses the update method found in the ClimbIOInputs class and passes down inputs as the parameter
        Logger.processInputs("Climb", inputs); // It will process the inputs in the logger and passess down "Climb" and inputs

        climbDisconnectedAlert.set(!inputs.climbConnected); //A alert will be set if it shows from the inputs that climb is not connected
    }

    public void setPosition(ClimbPosition pos) {
        climbIO.setClimbClosedLoop(Rotations.convertFrom(pos.value, Degrees)); //Converts the rotation to degree
        targetPosition = pos; // The target position is equal to the object that was passed down as a parameter
    }

    public double getAngle() {
        return inputs.climbPositionDeg; //Returns Climb positions in degrees
    }

    public ClimbPosition getTargetPosition() {
        return targetPosition; //Returns the target position that was created in the setPosition method
    }

    public boolean isAtPosition() {
        return isAtPosition(targetPosition); //If it is at the correct position it will return either true or false. 
    }

    public boolean isAtPosition(ClimbPosition pos) {
        return Math.abs(inputs.climbPositionDeg - pos.value) < ClimbConstants.climbTolerance;
    }
    
    @AutoLogOutput
    public boolean isFree() {
        return this.getCurrentCommand() == null;
    }

    @AutoLogOutput
    public String current() {
        return this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "NONE"; /* It will get the current command, if its not null/nothing
         then get the actual name of the command being used, if it is nothing then it will just say its none
        */
    }

    public void stopClimb() {
        climbIO.stopClimb(); //Uses the stop climb method found in the Climb IO class
    }

    public void resetState() {
        climbIO.resetState(); //Calls the reset state method 
        setPosition(ClimbPosition.START); //Sets the position to the start value

    }
}