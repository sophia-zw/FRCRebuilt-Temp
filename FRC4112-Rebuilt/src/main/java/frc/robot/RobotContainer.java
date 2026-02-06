// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  private final Drive drive;
	private final Intake intake;
	private final Indexer indexer;
	private final Shooter shooter;
	private final Vision vision;

  private final CommandXboxController controller = new CommandXboxController(0);

  private final AutoChooser autoChooser;
  private final LoggedDashboardChooser<Command> sysIdChooser;

  private final Field2d field;
  public RobotContainer() {
    configureBindings();


   	// Setup Command Usage Logging
		LoggedCommandScheduler.init(CommandScheduler.getInstance());

		// Set up dashboard choosers
		autoChooser = new AutoChooser();
		sysIdChooser = new LoggedDashboardChooser<>("SysId Choices");

    field = new Field2d();
		field.setRobotPose(drive.getFieldPose());
		SmartDashboard.putData("Field", field);
  }
  private void configureAutos() {
		SmartDashboard.putData("Auto Chooser", autoChooser);
		AutoRoutineFactory autos = new AutoRoutineFactory(drive, intake, indexer, shooter, climb);

		RobotModeTriggers.autonomous().whileTrue(
				Commands.runOnce(this::resetState).andThen(autoChooser.selectedCommandScheduler()));
	}

  public void update(){
    field.setRobotPose(drive.getFieldPose());
		LoggedCommandScheduler.periodic();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
				DriveCommands.joystickDrive(
						drive,
						() -> -controller.getLeftY(),
						() -> -controller.getLeftX(),
						() -> -controller.getRightX()));
    controller
				.b();
    controller
				.y();
    controller
				.x();
    controller
				.a();
    controller
				.start()
				.onTrue(
						Commands.runOnce(
								() -> drive.setPose(
										new Pose2d(drive.getFieldPose().getTranslation(), new Rotation2d())),
								drive)
								.ignoringDisable(true));
    //back buttons of the controller. Trigger is the furthest back
    controller
				.leftBumper();
    controller
				.leftTrigger();
    controller
				.rightBumper();
    controller
				.rightTrigger();

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public boolean isFree(Subsystem... subsystems) {
		for (Subsystem subsystem : subsystems) {
			if (subsystem.getCurrentCommand() != null) {
				return false;
			}
		}
		return true;
	}

  public void resetState() {
		intake.resetState();
	  shooter.resetState();
		indexer.resetState();
		climb.resetState();
	}

  private void configureSysId() {
		// Set up SysId routines
		sysIdChooser.addDefaultOption("None", Commands.none());
		// Drivetrain
		sysIdChooser.addOption(
				"Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
		sysIdChooser.addOption(
				"Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
		sysIdChooser.addOption(
				"Drive SysId (Quasistatic Forward)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Drive SysId (Quasistatic Reverse)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		sysIdChooser.addOption(
				"Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

		// Indexer
		sysIdChooser.addOption(
				"Indexer SysId (Quasistatic Forward)", indexer.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Indexer SysId (Quasistatic Reverse)", indexer.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		sysIdChooser.addOption(
				"Indexer SysId (Dynamic Forward)", indexer.sysIdDynamic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Indexer SysId (Dynamic Reverse)", indexer.sysIdDynamic(SysIdRoutine.Direction.kReverse));

		// Shooter
		sysIdChooser.addOption(
				"Shooter SysId (Quasistatic Forward)", shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Shooter SysId (Quasistatic Reverse)", shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		sysIdChooser.addOption(
				"Shooter SysId (Dynamic Forward)", shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Shooter SysId (Dynamic Reverse)", shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

		// Intake
		sysIdChooser.addOption(
				"Intake SysId (Quasistatic Forward)", intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Intake SysId (Quasistatic Reverse)", intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		sysIdChooser.addOption(
				"Intake SysId (Dynamic Forward)", intake.sysIdDynamic(SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(
				"Intake SysId (Dynamic Reverse)", intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));
	}

  public Command getSysIdCommand() {
		return sysIdChooser.get();
	}

  public Command cancel() {
		return Commands.runOnce(
						() -> {
							if (drive.getCurrentCommand() != null)
								drive.getCurrentCommand().cancel();
							if (intake.getCurrentCommand() != null)
								intake.getCurrentCommand().cancel();
							if (shooter.getCurrentCommand() != null)
								shooter.getCurrentCommand().cancel();
							if (indexer.getCurrentCommand() != null)
								indexer.getCurrentCommand().cancel();
              if(climb.getCurrentCommand() != null)
                climb.getCurrentCommand().cancel();
						});
	}
}


