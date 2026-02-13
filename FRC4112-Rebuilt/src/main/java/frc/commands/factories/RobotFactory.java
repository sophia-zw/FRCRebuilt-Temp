package frc.robot.commands.factories;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class RobotFactory {

    public static Command trackHub(Shooter s) {
        return Commands.sequence(
            Commands.parallel(
                ShooterFactory.setTurret(s),
                ShooterFactory.angle(s)
            )
        );
    }

    public static Command intakeFuel(Intake i) {
        return Commands.parallel(
            IntakeFactory.lowerIntake(i),
            IntakeIO.setWheels(IntakeConstants.wheelVoltage)
        );
    }
}
