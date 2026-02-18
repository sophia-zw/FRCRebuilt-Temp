package frc.robot.subsystems.elevator;

public class ElevatorConstants {


	public enum ElevatorPosition {



		public final double value;
        private ElevatorPosition(double value){
            this.value = value;
	}

	public static final double elevatorTolerance = 0.2;
	//change # later
	
	public static final TalonFXConfiguration elevConfig = new TalonFXConfiguration()
}