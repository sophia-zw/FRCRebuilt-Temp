package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Shooter.ShooterConstants.Constants;
import frc.robot.subsystems.Shooter.ShooterConstants.MotionState;
import frc.robot.subsystems.Shooter.ShooterConstants.ShooterStates;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import edu.wpi.first.math.filter.Debouncer;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Seconds; 
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase {
    private final ShooterIO io; 
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final Debouncer fuelDebouncer;

    private boolean hasFuel = false;
    private SysIdRoutine sysId;
    public Shooter(ShooterIO io) {
        this.io = io;
        //alerts

        fuelDebouncer = new Debouncer(0.1); //constant needs to be tweaked
        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(0.35),
                Volts.of(1),
                Seconds.of(5),
                state -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> runCharacterization(volts.in(Volts)), null, this)
        );
    }
}

public void periodic(){
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    //disconnected alerts set
}

public boolean isFree(){
    return this.getCurrentCommand() == null;
}

public void setShooterPosition(){

}

public void resetState(){
    //reset voltages and states
}

public String getCurrent(){
    return this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "NONE";
}

public void setShooter(){
    var output = 0; //change
    Logger.recordOutput("Shooter Voltage", 0);
}

public void stopShooter(){
    //change
    Logger.recordOutput("Shooter Voltage" , 0);
}
public void hasFuelStatus(boolean has){
    hasFuel = has;
}
public boolean hasNote(){
    return false; //change
}

public void runCharacterization(double volts){
    //change
}

public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
}
public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
}

