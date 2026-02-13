package frc.robot.subsystems.shooter;
import static edu.wpi.first.units.Units.Amps; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Amps
import static edu.wpi.first.units.Units.Degrees; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Degrees
import static edu.wpi.first.units.Units.DegreesPerSecond; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#DegreesPerSecond
import static edu.wpi.first.units.Units.Volts; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/Units.html#Volts

import com.ctre.phoenix6.BaseStatusSignal; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/BaseStatusSignal.html
import com.ctre.phoenix6.StatusSignal; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/StatusSignal.html

import com.ctre.phoenix6.controls.MotionMagicVoltage; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/controls/MotionMagicVoltage.html
import com.ctre.phoenix6.controls.VoltageOut; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/controls/VoltageOut.html
import com.ctre.phoenix6.hardware.ParentDevice; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/hardware/ParentDevice.html
import com.ctre.phoenix6.hardware.Pigeon2; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/hardware/Pigeon2.html
import com.ctre.phoenix6.hardware.TalonFX; //https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/hardware/TalonFX.html
import com.ctre.phoenix6.controls.Follower;
import au.grapplerobotics.LaserCan; //N/A
import edu.wpi.first.math.filter.Debouncer; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/filter/Debouncer.html

import edu.wpi.first.units.measure.Angle; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/Angle.html
import edu.wpi.first.units.measure.AngularVelocity; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/AngularVelocity.html
import edu.wpi.first.units.measure.Current; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/Current.html
import edu.wpi.first.units.measure.Voltage; //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/units/measure/Voltage.html
public class ShooterIOReal implements ShooterIO {
    private final TalonFX turret, fuel1, fuel2, angler, wheels; //1 is left, 2 is right
    private final Follower fuelFollower;

    private StatusSignal<Angle> anglerPos;
    private StatusSignal<Voltage> fuelVol, anglerCur, turretVol, wheelsVol;
    private StatusSignal<AngularVelocity> fuelVel, anglerVel, turrVel, wheelsVel;
    private StatusSignal<Current> fuelCur, anglerCur, turretCur, wheelsCur;

    private final Debouncer turretConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer fuelConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer anglerConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer wheelsConnectedDebouncer = new Debouncer(0.5); 

    //MOTION MAGIC VOLTAGE HASN'T BEEN IMPLEMENTED YET

    private final VoltageOut turretOut = new VoltageOut(0);
    private final VoltageOUt fuelOut = new VoltageOut(0);
    private final VoltageOut anglerOut = new VoltageOut(0);
    private final VoltageOut wheelsOut = new VoltageOut(0);
    public ShooterIOReal() {
        turret = new TalonFX(Ports.SHOOTER_TURRET);
        fuel1 = new TalonFX(Ports.SHOOTER_FUEL1);
        fuel2 = new TalonFX(Ports.SHOOTER_FUEL2);
        wheels = new TalonFX(Ports.SHOOTER_WHEELS);
        fuelFollower = new Follower(fuel1.getDeviceID(), false);
        angler = new TalonFX(Ports.SHOOTER_ANGLER);

        turret.getConfigurator().apply(ShooterConstants.turretConfig);
        fuel1.getConfigurator().apply(ShooterConstants.fuelConfig);
        fuel2.getConfigurator().apply(ShooterConstants.fuelConfig);
        angler.getConfigurator().apply(ShooterConstants.anglerConfig);
        wheels.getConfigurator().apply(ShooterConstants.wheelsConfig);

        turretCur = turret.getStatorCurrent();
        turretVol = turret.getMotorVoltage();
        turretVel = turret.getVelocity();
        
        fuelCur = fuel1.getStatorCurrent();
        fuelVol = fuel1.getMotorVoltage();
        fuelVel = fuel1.getVelocity();

        anglerCur = angler.getStatorCurrent();
        anglerVol = angler.getMotorVoltage();
        anglerVel = angler.getVelocity();
        anglerPos = angler.getPosition(); //questionable

        wheelsCur = wheels.getStatorCurrent();
        wheelsVol = wheels.getMotorVoltage();
        wheelsVel = wheels.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(50, turretCur, anglerCur, fuelCur, turretVol, fuelVol, anglerVol, fuelVel, turretVel, anglerVel, anglerPos, wheelsCur, wheelsVel, wheelsVol);
        ParentDevice.optimizeBusUtilizationForAll(turret, fuel1, fuel2, angler, wheels);

        fuel2.setControl(fuelFollower);
    }
    public void updateInputs(ShooterIOInputs io) {
        var turretStatus = BaseStatusSignal.refreshAll(turretCur, turretVel, turretVol);
        var fuelStatus = BaseStatusSignal.refreshAll(fuelCur, fuelVel, fuelVol);
        var anglerStatus = BaseStatusSignal.refreshAll(anglerCur, anglerVel, anglerVol, anglerPos);
        var wheelsStatus = BaseStatusSignal.refreshAll(wheelsCur, wheelsVol, wheelsVel);
        
        io.turretConnected = turretConnectedDebouncer.calculate(turretStatus.isOK());
        io.anglerConnected = anglerConnectedDebouncer.calculate(anglerStatus.isOK());
        io.fuelConnected = fuelConnectedDebouncer.calculate(fuelStatus.isOK());
        io.fuel2Connected = fuel2.isConnected();
        io.wheelsConnected = wheelsConnectedDebouncer.calculate(wheelsStatus.isOK());

        io.turretVelocityDegPerSec = turretVel.getValue().in(DegreesPerSecond);
        io.turretVoltage = turretVol.getValue().in(Volts);
        io.turretCurrent = turretCur.getValue().in(Amps);

        io.fuelVelocityDegPerSec = fuelVel.getValue().in(DegreesPerSecond);
        io.fuelVoltage = fuelVol.getValue().in(Volts);
        io.fuelCurrent = fuelCur.getValue().in(Amps);

        //Questionable io.anglerPositionDeg = anglerPos.getValue().in(Degrees);
        io.anglerVelocityDegPerSec = anglerVel.getValue().in(DegreesPerSecond);
        io.anglerVoltage = anglerVol.getValue().in(Volts);
        io.anglerCurrent = anglerCur.getValue().in(Amps);

        io.wheelsVelocityDegPerSec = wheelsVel.getValue().in(DegreesPerSecond);
        io.wheelsVoltage = wheelsVol.getValue().in(Volts);
        io.wheelsCurrent = wheelsCur.getValue().in(Amps);
    }
    
    @Override   
    public void setTurretClosedLoop(double value){
        turret.setControl(turretOut.withPosition(value));
    }

    @Override
    public void setTurretOpenLoop(double value){
        turret.setVoltage(value);
    }

    @Override
    public void setAnglerClosedLoop(double value){
        angler.setControl(anglerOut.withPosition(value));
    }

    @Override
    public void setAnglerOpenLoop(double value){
        angler.setVoltage(value);
    }
/*
    public void setTurretVoltage(double value){
        turret.setControl(turretOut.withOutput(value));
    }
*/
    @Override
    public void setFuelVoltage(double value){
        fuel.setControl(fuelOut.withOutput(value));
    }

    @Override
    public void setWheelsVoltage(double value){
        wheels.setControl(wheelsOut.withOutput(value));
    }

    /*
    @Override
    public void resetState(){
        /*turret.setPosition
        angler.setPosition

        both of these need a pigeon which i'll deal with later
        
    }
    */
    /*
    @Override
    public void syncPigeon(){
        turret.setPosition
        angler.setPosition

        both of these need a pigeon which i'll deal with later
        
    }
    */

}
