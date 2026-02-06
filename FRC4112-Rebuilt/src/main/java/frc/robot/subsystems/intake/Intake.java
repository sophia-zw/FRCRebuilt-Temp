package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.intake.IntakeConstants.IntakePosition;

public class Intake extends SubsystemBase {
    
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final Alert pivotDisconnectedAlert, wheelsDisconnectedAlert, indexerDisconnectedAlert; 
    /*Don't know about if there is a laserCAN, but any sensors also need an alert
    We have two motors based on the CAD. The right one (front side view) should be the wheels, the left should be the pivot (up and down)
    */
    private final Debouncer stuckFuel;
    private final Timer stuckCooldownTimer;
    @AutoLogOutput
    private IntakePosition targetPosition;

    private final SysIdRoutine sysId;

    public Intake(IntakeIO io) {
        this.io = io;
        pivotDisconnectedAlert = new Alert("Disconnected intake pivot motor.", AlertType.kError);
        wheelsDisconnectedAlert = new Alert("Disconnected intake wheels motor.", AlertType.kError);
        indexerDisconnectedAlert = new Alert("Disconnected intake indexer motor.", AlertType.kError);
        //Again laserCAN state is unknown

        /*Sensor stuff */
        stuckFuel = new Debouncer(0.5, DebounceType.kRising);
        stuckCooldownTimer = new Timer();
        stuckCooldownTimer.start();


        targetPosition = IntakePosition.LOWERED;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(0.3),
                        Volts.of(1.5),
                        Seconds.of(5),
                        (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        pivotDisconnectedAlert.set(!inputs.pivotConnected);
        wheelsDisconnectedAlert.set(!inputs.wheelsConnected);
        indexerDisconnectedAlert.set(!inputs.indexerConnected);
        //Again laserCAN state is unknown
    }

    public void setIntakePosition(IntakePosition position) {
        io.setPivotClosedLoop(position);
        targetPosition = position;
    }

    public IntakePosition getIntakePosition() {
        return targetPosition;
    }

    @AutoLogOutput
    public boolean isAtPosition() {
        return isAtPosition(targetPosition);
    }

    public boolean isAtPosition(IntakePosition position) {
        return Math.abs(inputs.pivotPositionDeg - position.value) < IntakeConstants.pivotTolerance;
    }

    public void lowerIntake() {
        io.setPivotClosedLoop(IntakePosition.LOWERED);
        io.setWheels(IntakeConstants.wheelVoltage);
        targetPosition = IntakePosition.LOWERED;
    }

    public void purgeIntake() {
        io.setPivotClosedLoop(IntakePosition.LOWERED);
        io.setWheels(-IntakeConstants.wheelVoltage);
        targetPosition = IntakePosition.LOWERED;
    }

    public void retractIntake() {
        io.setPivotClosedLoop(IntakePosition.RETRACTED);
        io.setWheels(0.0);
        targetPosition = IntakePosition.RETRACTED;
    }
    //This is for the indexer but it seems more independent from the intake, so if you want to create a separate organization this is the code
    /*
    public void runIndexer() {
        io.setIndexer(IntakeConstants.indexerVoltage);
    }

    public void reverseIndexer() {
        io.setIndexer(-IntakeConstants.indexerVoltage);
    }

    public void stopIndexer() {
        io.setIndexer(0.0);
    }
    */

    public void setWheels(double output) {
        io.setWheels(output);
    }

    public void setWheels() {
        io.setWheels(IntakeConstants.wheelVoltage);
    }


    public void resetState() {
        io.resetState();
        io.setPivotClosedLoop(IntakePosition.START);
        io.setWheels(0.0);
        targetPosition = IntakePosition.START;
    }

    /*IDK if we'll have a sensor but if we do have a laserCAN, these have been adjusted for fuel */
    @AutoLogOutput
    public boolean fuelIsThere() {
        return inputs.sensorDistanceMillimeters <= IntakeConstants.minSensorDistance;
    }

    @AutoLogOutput
    public boolean stuckFuel() {
        if (stuckCooldownTimer.get() < IntakeConstants.stuckFuelTimeSeconds) {
            return stuckFuel.calculate(false);
        }
        return stuckFuel.calculate(fuelIsThere());
    }
    
    @AutoLogOutput
    public boolean isFree() {
        return this.getCurrentCommand() == null;
    }

    @AutoLogOutput
    public String current() {
        return this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "NONE";
    }

    public void resetTimer() {
        stuckCooldownTimer.reset();
        stuckCooldownTimer.start();
    }

    private void runCharacterization(double volts) {
        io.setPivotOpenLoop(volts);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction));
    }
}