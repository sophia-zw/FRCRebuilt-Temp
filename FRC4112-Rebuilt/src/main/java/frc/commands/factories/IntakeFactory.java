//I THINK I CAN NIKHILA CHECK THIS  :)

package frc.robot.commands.factories;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakePosition;

public class IntakeFactory {
    public static Command lowerIntake(Intake intake, Supplier<IntakePosition> pos) {
        return new InstantCommand(() -> intake.lowerIntake()) // questioning if wheels are running while intake is lowering. if so, no need to call any setWheels method to make wheels run, else would have to do so. 
            .andThen(new WaitUntilCommand(() -> intake.isAtPosition(pos.get())))
            .andThen(new WaitUntilCommand(() -> intake.fuelIsThere()).withTimeout(0.5)) // if no fuel withiin 0.5 seconds, commandscheduler will interrupt command so it doesnt run forever
            .andThen(new InstantCommand(() -> intake.setWheels()))// sets wheels to the wheel voltage to intake if previous command is satisfied
            .andThen(new WaitUntilCommand(() -> !intake.fuelIsThere()).withTimeout(0.5))
            .andThen(new InstantCommand(() -> intake.setWheels(0.0))) 
            .withName("Lower Intake");
    }
    public static Command purgeIntake(Intake intake) {
        return new InstantCommand(() -> intake.purgeIntake()) // already lowers intake so no need for extra lower intake command
            .andThen(new WaitUntilCommand(() -> !intake.stuckFuel()).withTimeout(2.0)) // to keep running until ttheres nothinig stuck, but stops after 2 seconds in case fuel is jammed/command runs forever
            .andThen(new InstantCommand(() -> intake.setWheels(0.0)))
            .withName("Purge Intake");
    }
    public static Command retractIntake(Intake intake) {
        return new InstantCommand(() -> intake.retractIntake()) // already does all actions to go to retract position + stop wheels
            .withName("Retract Intake");
    }
    public static Command moveIntake(Intake intake, Supplier<IntakePosition> pos) { // move intake to specific position
        return new InstantCommand(() -> intake.setIntakePosition(pos.get()))
            .andThen(new WaitUntilCommand(() -> intake.isAtPosition(pos.get())))
            .withName("Move Intake");
    }

    
}


