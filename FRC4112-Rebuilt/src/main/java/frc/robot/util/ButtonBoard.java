package frc.robot.util;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.ScorePositionsUtil.ScoringPosition;

/**
 * This class is a wrapper for the button board. It is used to interface with
 * the button board and update its outputs.
 * 
 * The button board is a custom board with buttons that are used.

 * Button 1 is used to 
 * Button 2 is used to 
 * Button 3 is used to 
 * Button 4 is used to 
 * Button 5 is used to 
 * Button 6 is used to 
 * 
 * The button board also has 11 LEDs that are used.
 * LEDs 1-6 are used to indicate the sector that the robot is in.
 * LED 7 is used to indicate the robot is in a valid 
 * LED 8 is used to indicate the robot is in a valid 
 * LED 9 is used to indicate the robot is in a valid 
 * state.
 * LED 10 is used to indicate the robot is in a valid 
 * LED 11 is used to indicate the robot is in a valid 
 */
public class ButtonBoard extends CommandGenericHID {
    @AutoLogOutput(key = "ButtonBoard/StoredTargetPosition")
    private ScoringPosition targetPosition = ScoringPosition.NONE;
    /*
     * First array is desired conditions, second is a mask for checked conditions
     * 0b10000: 
     * 0b01000: 
     * 0b00100: 
     * 0b00010: 
     * 0b00001: 
     * Index is for LED i+7 output
     */

    private static final int[][] conditions = {
            { 0b11100, 0b11000, 0b01011, 0b11111, 0b01010 },
            { 0b11110, 0b11010, 0b00000, 0b00000, 0b01010 } };

    public ButtonBoard(int port) {
        super(port);
    }
/*
    // Following method must be integrated into the vision subsystem periodic
    public void update(int sector, boolean systemsFree, boolean hasCoralLoaded, boolean hasAlgae) {
        pollButtons();
        // Gets state as binary
        int state = (sector < 6 ? 0b1 : 0b0) << 4 |
                (systemsFree ? 0b1 : 0b0) << 3 |
                (hasCoralLoaded ? 0b1 : 0b0) << 2 |
                (hasAlgae ? 0b1 : 0b0) << 1 |
                (sector == 7 ? 0b1 : 0b0) << 0;
        // Sets output bits comparing state to conditions and adding output for sector
        // by shifting it into the correct position
        int out = ((state & conditions[1][4]) == conditions[0][4] ? 0b1 << 10 : 0b0) |
                ((state & conditions[1][3]) == conditions[0][3] ? 0b1 << 9 : 0b0) |
                ((state & conditions[1][2]) == conditions[0][2] ? 0b1 << 8 : 0b0) |
                ((state & conditions[1][1]) == conditions[0][1] ? 0b1 << 7 : 0b0) |
                ((state & conditions[1][0]) == conditions[0][0] ? 0b1 << 6 : 0b0) |
                (sector < 6 ? 0b1 << sector : 0b0); // Fills 0 to 5
        // Puts current alliance in buffer as bits 15 and 16 to set hexagon color.
        out |= (DriverStation.getAlliance().isPresent()
                ? (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 0b10 : 0b01)
                : 0b0) << 14;
        out |= DriverStation.isEnabled() ? 0b0 : 0b111111; // Sets all LEDs to on if connected and disabled
        Logger.recordOutput("ButtonBoard/LEDState", Integer.toBinaryString(out));
        getHID().setOutputs(out);
    }

    private void pollButtons() {
        Logger.recordOutput("ButtonBoard/ScorePressed", checkScorePressed());
        Logger.recordOutput("ButtonBoard/AlgaeGrabPressed", getHID().getRawButton(9));
        Logger.recordOutput("ButtonBoard/ScoreProcessorPressed", getHID().getRawButton(10));
        Logger.recordOutput("ButtonBoard/ScoreNetPressed", getHID().getRawButton(11));
        Logger.recordOutput("ButtonBoard/EjectAlgaePressed", getHID().getRawButton(12));
        Logger.recordOutput("ButtonBoard/CancelPressed", getHID().getRawButton(13));
        Logger.recordOutput("ButtonBoard/HoldPressed", getHID().getRawButton(14));
    }
*/

    private boolean checkScorePressed() {
        boolean pressed = false;
        for (int i = 1; i < 9; i++) {
            if (getHID().getRawButton(i)) {
                targetPosition = ScoringPosition.values()[i - 1];
                pressed = true;
            }
        }
        return pressed;
    }

    public Trigger scorePressed(EventLoop loop) {
        // Returns the trigger and sets the stored position to NONE after the command is
        // run
        return new Trigger(loop, this::checkScorePressed).onFalse(new InstantCommand(this::getStoredPosition));
    }
/*
    public Trigger algaeGrabPressed(EventLoop loop) {
        return new Trigger(loop, () -> getHID().getRawButton(9));
    }

    public Trigger scoreProcessorPressed(EventLoop loop) {
        return new Trigger(loop, () -> getHID().getRawButton(10));
    }

    public Trigger scoreNetPressed(EventLoop loop) {
        return new Trigger(loop, () -> getHID().getRawButton(11));
    }

    public Trigger ejectAlgaePressed(EventLoop loop) {
        return new Trigger(loop, () -> getHID().getRawButton(12));
    }

    public Trigger cancelPressed(EventLoop loop) {
        return new Trigger(loop, () -> getHID().getRawButton(13));
    }

    public Trigger holdPressed(EventLoop loop) {
        return new Trigger(loop, () -> getHID().getRawButton(14));
    }

    public Trigger scorePressed() {
        return scorePressed(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger algaeGrabPressed() {
        return algaeGrabPressed(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger scoreProcessorPressed() {
        return scoreProcessorPressed(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger scoreNetPressed() {
        return scoreNetPressed(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger ejectAlgaePressed() {
        return ejectAlgaePressed(CommandScheduler.getInstance().getDefaultButtonLoop());
    }
*/
    public Trigger cancelPressed() {
        return cancelPressed(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger holdPressed() {
        return holdPressed(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public ScoringPosition getStoredPosition() {
        var temp = targetPosition;
        targetPosition = ScoringPosition.NONE;
        return temp;
    }
}