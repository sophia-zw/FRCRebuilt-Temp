package frc.robot.commands.factories;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPosition;
public class ElevatorFactory {
    
    public static Command setElevatorHeight(Elevator elev, Supplier<ElevatorPosition> pos){
        return new InstantCommand(() -> elev.setPosition(pos.get()))
                .andThen(new WaitUntilCommand(() -> elev.isAtPosition(pos.get())))
                .withName("SetElevatorHeight");
    }    
}