package frc.robot.commands.factories;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class RobotFactory {

    //this will be a toggle, so that it will constantly track the hub
    public static Command trackHub(Shooter s, Supplier<AnglerPosition> ang) {
        return Commands.run(() -> {
            S.setAnglerPosition(ang.get());
            s.setTurretPosition();
        }, s)
        .withName("Track Hub");
    }

    //only intakes if the intake is lowered, otherwise lowers the intake and then sets the wheels to intake
    public static Command intakeFuel(Intake i) {
        return Commands.deadline(
            Commands.either(
                Commands.run(i.setWheels(), i),
                Commands.sequence(
                    i.lowerIntake(),
                    Commands.waitUntil(i.isAtPosition(IntakePosition.LOWERED)),
                    i.setWheels()
                ),
                () -> i.isAtPosition(IntakePosition.LOWERED)
            ),
            Commands.run(() -> {
                if (i.fuelIsThere()) {
                    i.setWheels(-IntakeConstants.wheelVoltage);
                }}, i
            )
        )
        .finallyDo(() -> i.setWheels(0.0))
        .withName("Intake Fuel");
    }

 //shooter should have different voltage constants depending on what preset shooter position we are at, this will be a held button or toggle
    public static Command shootFuel(Shooter s, Indexer i, Supplier<turretVoltage> tVoltage) {
        return Commands.run(() -> {
            s.setShooter(tVoltage.get());
            s.setWheels();
            i.setWheels();
        }, s)
        .finallyDo(() -> {
            s.stopShooter();
            s.stopWheels();
            i.stopIndexer();
        })
        .withName("Shoot Fuel");
    }

    //should add eject shooter here
}
