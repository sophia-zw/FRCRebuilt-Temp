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
    private TurretPosition targetDir = TurretPosition.START;
    private AnglerPosition targetPos = AnglerPosition.START;

    private boolean hasFuel = false;
    private SysIdRoutine sysId;
    public Shooter(ShooterIO io) {
        this.io = io;
        turretDisconnectedAlert = new Alert("Disconnected arm turret motor");
        fuel1DisconnectedAlert = new Alert("Disconnected arm fuel1 (left) motor alert");
        fuel2DisconnectedAlert = new Alert("Disconnected arm fuel2 (right) motor alert");
        anglerDisconnectedAlert = new Alert("Disconnected arm angler motor alert")

        
        fuelDebouncer = new Debouncer(0.1); //constant needs to be tweaked
        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(0.35),
                Volts.of(1),
                Seconds.of(5),
                state -> Logger.recordOutpu("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> runCharacterization(volts.in(Volts)), null, this)
        );
    }
}

public void periodic(){
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    turretDisconnectedAlert.set(!inputs.turretConnected);
    fuel1DisconnectedAlert.set(!inputs.fuelConnected);
    fuel2DisconnectedAlert.set(!inputs.fuel2Connected);
    anglerDisconnectedAlert.set(!inputs.anglerConnected);
}

public boolean isFree(){
    return this.getCurrentCommand() == null;
}

//I'm just gonna write this if we have to resort to putting a compass rather than calculating
public void setTurretPosition(TurretPosition dir){
    io.setTurretClosedLoop(dir.value);
    targetDir = dir;
}

public void setAnglerPosition(AnglerPosition anglePos){
    io.setAnglerClosedLoop(anglePos.value);

    targetPos = anglePos;
}

public boolean isAtPosition(){
    return isAtPosition(targetPos);
}

public boolean isAtPosition(AnglerPosition anglePos){
    return false; //add code
}

/*
public void setShooterPosition(){

}
*/

public AnglerPosition getTargetPos(){
    return targetPos;
}

public TurretPosition getTargetDir(){
    return targetDir;
}

public String getCurrent(){
    return this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "NONE";
}

public void setShooter(){
    io.setFuelVoltage(0); //change to a set constant
    Logger.recordOutput("Shooter Voltage", 0);
}

public void stopShooter(){
    io.setFuelVoltage(0);
    Logger.recordOutput("Shooter Voltage" , 0);
}

public void resetState(){
    io.resetState();
    setAnglerPosition(AnglerPosition.START);
    setTurretPosition(TurretPosition.START);
}

public void hasFuelStatus(boolean has){
    hasFuel = has;
}
public boolean hasFuel(){
    return false; //change
}

public void runCharacterization(double voltsT, double voltsA){
    //VoltsT and VoltsA might be the same
    io.setTurnOpenLoop(voltsT);
    io.setAnglerOpenLoop(voltsA);
}

public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
}
public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
}

