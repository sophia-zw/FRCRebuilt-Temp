//I THINK I CAN NIKHILA CHECK THIS  :)

package frc.robot.commands.factories;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakePosition;

public class IntakeFactory {
//One parameter, the intake subsystem
    public static Command lowerIntake(Intake intake) {
    /*The Commands.sequence method runs listed commands one after another.*/
        return Commands.sequence(
        /**runOnce() runs a action and finishes
         * public static Command runOnce(Runnable action, Subsystem... requirements)
         * We call the lowerIntake from the Intake.java file, this is the first parameter. The second is the subsystem we need, which is the intake.
         *
         * public void lowerIntake() {
                io.setPivotClosedLoop(IntakePosition.LOWERED); 
                io.setWheels(IntakeConstants.wheelVoltage); 
                targetPosition = IntakePosition.LOWERED; 
        
            :: is just a shortcut for lambda, when we don't have a parameter.
         */
            Commands.runOnce(intake::lowerIntake, intake),
        /**After we've lowered the Intake by setting the voltage to that constant we want to run the indexer to intake coral. Runs and finishes
         * We call the runIndexer from the Intake.java file, this is the first parameter. The second is the subsystem we need, which is the intake.
         *
         * public void runIndexer() {
                io.setIndexer(IntakeConstants.indexerVoltage); //We set the indexer with a certain voltage and it runs.
         */
            Commands.runOnce(() -> intake.runIndexer(), intake),
        /*Then we wait Until the coralIsThere command returns a true.  We want to make sure we've intaken a coral until we do anything with said "coral"
        We call the coralIsThere from the Intake.java file

        public boolean coralIsThere() {
            return inputs.sensorDistanceMillimeters <= IntakeConstants.minSensorDistance;
        :: is just a shortcut for lambda, when we don't have a parameter.
        Then we name this command LowerIntake.
        */
            Commands.waitUntil(intake::coralIsThere)
        ).withName("LowerIntake");
    }

//______________________________________________________________________________________________________________________________________
//One parameter, the intake subsystem
    public static Command purgeIntake(Intake intake) {
        /*The Commands.sequence method runs listed commands one after another.*/
        return Commands.sequence(
            /*runOnce() runs the group of actions and finishes */
            Commands.runOnce(() -> {
                /** We need to output anything currently in the intake, so we reverse the indexer. Calling reverseIndexer() in Intake.java
                 * 
                 * public void reverseIndexer() {
                        io.setIndexer(-IntakeConstants.indexerVoltage);//We set the indexer with a negative voltage and it runs in the opposite direction since the relative points are reversed.
                    }
                We lower the intake and then use the wheels to shoot out the piece. Our target Position is lowered and we want to test later if it really went down.

                    //Removes anything from the intake aka shoots it out. This may be used when coral gets stuck or comes in at a awkard angle.
                   public void purgeIntake() {
                        io.setPivotClosedLoop(IntakePosition.LOWERED); 
                        io.setWheels(-IntakeConstants.wheelVoltage); 
                        targetPosition = IntakePosition.LOWERED;
                    
                    Now we set the position to lowered because we want to attempt to grab another game piece.
                     * Sets the intake control to the desired position.  @param position The desired position to set the intake to.
                   public void setIntakePosition(IntakePosition position) {
                        io.setPivotClosedLoop(position);
                        targetPosition = position; 
    }
                */
                intake.reverseIndexer();
                intake.purgeIntake();
                intake.setIntakePosition(IntakePosition.LOWERED);
            }, intake)
        //Then we name this command PurgeIntake.
        ).withName("PurgeIntake");
    }
//One parameter, the intake subsystem
    public static Command retractIntake(Intake intake) {
        /** runOnce() runs the action and finishes 
         * :: is just a shortcut for lambda, when we don't have a parameter.
         
         * The intake gets set to a retracted position, and then we stop the intake wheels so we aren't intaking or outaking anything.
         * public void retractIntake() {
                io.setPivotClosedLoop(IntakePosition.RETRACTED); 
                io.setWheels(0.0); 
                targetPosition = IntakePosition.RETRACTED;
    }

        //Then we name this command RetractIntake.
        */
        return Commands.runOnce(intake::retractIntake, intake).withName("RetractIntake");
    }
//Two parameters, the intake subsystem and a supplier, just do this to make things dynamic, which is position. Position includes the whole enum and all of our set positions like LOWERED, STRAIGHT,AVOID
    public static Command moveIntake(Intake intake, Supplier<IntakePosition> pos) {
        /* runOnce() runs the action and finishes

        * Sets the intake control to the desired position. @param position The desired position to set the intake to.

        public void setIntakePosition(IntakePosition position) {
            io.setPivotClosedLoop(position);
            targetPosition = position; 
    }
         */
        return Commands.runOnce(() -> intake.setIntakePosition(pos.get()), intake)
        /* Okay we've set the Intake to a position we want, but we need to wait until the intake is at the position because that initself takes some time.
        So and then we wait until() its at the position, so we call the method isAtPosition() in Intake.java
        
        public boolean isAtPosition() { 
            return isAtPosition(targetPosition); 
        }

        public boolean isAtPosition(IntakePosition position) { //METHODS CAN HAVE THE SAME NAME JUST HAS TO BE DIFFERENT PARAMETERS
            return Math.abs(inputs.pivotPositionDeg - position.value) < IntakeConstants.pivotTolerance;
        }

        We wait until it returns true.
        */
                .andThen(Commands.waitUntil(intake::isAtPosition))
                //Then we name this command MoveIntake.
                .withName("MoveIntake");
    }

    
}
