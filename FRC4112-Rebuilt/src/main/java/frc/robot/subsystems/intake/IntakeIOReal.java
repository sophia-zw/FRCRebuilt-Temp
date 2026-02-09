package frc.robot.subsystems.intake;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.intake.IntakeConstants.IntakePosition;
public class IntakeIOReal implements IntakeIO {
    private final TalonFX pivot;
    private final TalonFX wheels;
    private final LaserCan lasercan; //SENSOR STUFF

    private final MotionMagicVoltage positionVoltageRequest = new MotionMagicVoltage(0.0);

    private final StatusSignal<Angle> pivotAngle;
    private final StatusSignal<AngularVelocity> pivotVel;
    private final StatusSignal<Voltage> pivotAppliedVolts;
    private final StatusSignal<Current> pivotCurrent;

    private final StatusSignal<AngularVelocity> wheelVel;
    private final StatusSignal<Voltage> wheelAppliedVolts;
    private final StatusSignal<Current> wheelCurrent;

    private final Debouncer pivotConnectedDebounce = new Debouncer(0);
    private final Debouncer wheelConnectedDebounce = new Debouncer(0);
    
    public IntakeIOReal() {
        pivot = new TalonFX(Ports.INTAKE_PIVOT);
        wheels = new TalonFX(Ports.INTAKE_WHEEL);
        
        lasercan = new LaserCan(Ports.INDEXER_SENSOR); //SENSOR STUFF

        pivot.getConfigurator().apply(IntakeConstants.pivotConfig);
        wheels.getConfigurator().apply(IntakeConstants.wheelConfig);
        
        pivotAngle = pivot.getPosition();
        pivotVel = pivot.getVelocity();
        pivotAppliedVolts = pivot.getMotorVoltage();
        pivotCurrent = pivot.getStatorCurrent();

        wheelVel = wheels.getVelocity();
        wheelAppliedVolts = wheels.getMotorVoltage();
        wheelCurrent = wheels.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, pivotAngle, pivotVel, pivotAppliedVolts, pivotCurrent, wheelVel, wheelAppliedVolts, wheelCurrent);
        ParentDevice.optimizeBusUtilizationForAll(pivot, wheels);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        var pivotStatus = BaseStatusSignal.refreshAll(pivotAngle, pivotVel, pivotAppliedVolts, pivotCurrent);
        var wheelStatus = BaseStatusSignal.refreshAll(wheelVel, wheelAppliedVolts, wheelCurrent);

        inputs.pivotConnected = pivotConnectedDebounce.calculate(pivotStatus.isOK());
        inputs.pivotPositionDeg = pivotAngle.getValue().in(Degrees);
        inputs.pivotVelocityDegPerSec = pivotVel.getValue().in(DegreesPerSecond);
        inputs.pivotAppliedVolts = pivotAppliedVolts.getValue().in(Volts);
        inputs.pivotCurrent = pivotCurrent.getValue().in(Amps);
        
        inputs.wheelsConnected = wheelConnectedDebounce.calculate(wheelStatus.isOK());
        inputs.wheelsVelocityDegPerSec = wheelVel.getValue().in(DegreesPerSecond);
        inputs.wheelsAppliedVolts = wheelAppliedVolts.getValue().in(Volts);
        inputs.wheelsCurrent = wheelCurrent.getValue().in(Amps);

        //SENSOR STUFF
        var mes = lasercan.getMeasurement();
        inputs.sensorDistanceMillimeters = mes == null || mes.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT ? 1000 : mes.distance_mm;
        inputs.laserCANConnected = mes!=null;
        inputs.laserCANStatus = mes == null ? -1 : mes.status;
    }

    @Override
    public void setPivotClosedLoop(IntakePosition pos) {
        pivot.setControl(positionVoltageRequest.withPosition(Rotations.convertFrom(pos.value, Degrees)));
    }

    @Override
    public void setPivotOpenLoop(double volts) {
        pivot.setVoltage(volts);
    }

    @Override public void setWheels(double output) {
        wheels.setVoltage(output);
    }

    @Override
    public void resetState() {
        pivot.setPosition(Rotations.convertFrom(IntakePosition.START.value, Degrees));
    }
}