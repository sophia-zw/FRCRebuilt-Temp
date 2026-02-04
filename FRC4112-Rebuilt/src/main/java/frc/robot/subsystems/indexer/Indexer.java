package frc.robot.subsystems.indexer;

import edu.wpi.first.math.filter.Debouncer; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/filter/Debouncer.html
import edu.wpi.first.math.filter.Debouncer.DebounceType; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/filter/Debouncer.DebounceType.html
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.Alert; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/Alert.html
import edu.wpi.first.wpilibj.Timer; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/Timer.html
import edu.wpi.first.wpilibj.Alert.AlertType; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/Alert.AlertType.html
import edu.wpi.first.wpilibj2.command.Command; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html
import edu.wpi.first.wpilibj2.command.SubsystemBase; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SubsystemBase.html
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/sysid/SysIdRoutine.html


public class Indexer extends SubsystemBase {

    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final Alert indexerDisconnectedAlert; 

    public Indexer(IndexerIO io) {
        this.io = io

        indexerDisconnectedAlert = new Alert("Disconnected intake indexer motor.", AlertType.kError);

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(0.3),
                        Volts.of(1.5),
                        Seconds.of(5),
                        (state) -> Logger.recordOutput("Indexer/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }
    
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        indexerDisconnectedAlert.set(!inputs.indexerConnected);
    }

    public void runIndexer() {
        io.setIndexer(IntakeConstants.indexerVoltage);
    }

    public void reverseIndexer() {
        io.setIndexer(-IntakeConstants.indexerVoltage);
    }

    public void stopIndexer() {
        io.setIndexer(0.0);
    }
    
    public void runCharacterization(double volts){
        // what to add here. need to add more methods in io.java
    }
    
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
            return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }
}

// ADD IMPORTS
