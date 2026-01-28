package frc.robot.commands.factories;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants.ClimbPosition;

public class ClimbFactory {
    //Lambdas take in a parameter and return a action
    public static Command moveClimbToPosition(Climb c, Supplier<ClimbPosition> pos) {
        //Has two parameters 
        return Commands.sequence(
            
            Commands.runOnce(() -> c.setPosition(pos.get()), c),
            //Runs it once and allow to set it in a certain position
            Commands.waitUntil(() -> c.isAtPosition())
            //Waits until it reaches a certain position
        ).withName("MoveClimbToPosition");
    }
}