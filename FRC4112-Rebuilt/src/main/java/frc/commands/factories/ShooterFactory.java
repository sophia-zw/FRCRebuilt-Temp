package frc.factories;

import java.time.Instant;
import java.util.function.Supplier;
import frc.robot.subsystems.Shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOInputs;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.ShooterConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShooterFactory {
    

    public Command wheelsSupply(Shooter shooter, Indexer indexer){
        return new WaitUntilCommand(() -> indexer.hasFuel())
        .andThen(new InstantCommand(() -> shooter.setWheels())) 
        .andThen(new WaitUntilCommand(() -> !indexer.hasFuel()))
        .andThen(new WaitCommand(0.1))
        .andThen(() -> shooter.stopWheels())
        .withName("Set Wheels Fuel Up");
    }

    public Command angle(Shooter shooter, Supplier<AnglerPosition> ang){
        return new InstantCommand(() -> shooter.setAnglerPosition(ang.get())) 
        .andThen(new WaitUntilCommand(() -> shooter.isAtPosition(ang.get())))
        .andThen(Commands.waitSeconds(0.1))
        .withName("Set Angler Position");
    }


    public Command shoot(Shooter shooter, Supplier<AnglerPosition> ang, Supplier<TurretPosition> dir){
        return new WaitUntilCommand(() -> shooter.hasFuel())
        .andThen(new WaitUntilCommand(() -> angle(shooter, ang.get()))) 
        .andThen(new WaitUntilCommand(() -> setTurret(shooter, dir.get()))) 
        .andThen(new WaitCommand(0.1))
        .andThen(new InstantCommand(() -> shooter.setShooter())) 
        .andThen(WaitUntilCommand(() -> !shooter.hasFuel()))
        .andThen(new InstantCommand(() -> shooter.stopShooter()))
        .withName("Shoot");
    }
    public Command setTurret(Shooter shooter, Suppler<TurretPosition> dir){
        return new InstantCommand(() -> shooter.setTurretPosition(dir.get()))
        .andThen(new WaitUntilCommand(() -> shooter.isAtDirection(dir.get())))
        .andThen(new WaitCommand(0.1))
        .withName("Set Turret Direction");
    }

}
