package frc.robot.configuration;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.auton.framework.basic.SwerveAuton;

import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public enum Auton {
	
	NONE(
		"None (No Auton)",
		robot -> new WaitCommand(0)
	),
	
	CROSS_THE_LINE(
		"Battle of the Five Armies (No Note - Cross the Line)",
		robot -> new SwerveAuton(
			robot,
			0.35,
			0,
			robot.swerve.getFieldRelativeHeadingRotation2d()
		).withTimeout(2)
	),
	
	ONE_NOTE_NO_LINE_CROSS(
		"Fellowship of the Ring (One Note - No Line Cross)",
		robotContainer -> {
			
			RobotContainer.Commands robot = robotContainer.commands;
			
			return robot.waitToStartAuton()
				.andThen(robot.shootBelliedUpToSubwoofer());
			
		}
	),
	
	ONE_NOTE_WITH_LINE_CROSS("One Note - Cross Line", robotContainer -> {
		
		RobotContainer.Commands robot = robotContainer.commands;
		Supplier<Alliance> getAlliance = () ->
			DriverStation.getAlliance().orElse(Auton.getDefaultAlliance());
		
		Supplier<Command> blueAmpRedSource = () ->
			robot.shootBelliedUpToSubwoofer()
				.andThen(robot.waitToRolloutInAuton())
				.andThen(robotContainer.swerve.commands.driveForTime(
					Degrees.of(-60),
					0.35,
					Degrees.of(0),
					Seconds.of(1.5)
				));
		
		Command middle = robot.shootBelliedUpToSubwoofer()
			.andThen(robot.waitToRolloutInAuton())
			.andThen(robotContainer.swerve.commands.driveForTime(
				Degrees.of(0),
				0.35,
				Degrees.of(0),
				Seconds.of(1)
			));
		
		Supplier<Command> blueSourceRedAmp = () ->
			robot.shootBelliedUpToSubwoofer()
				.andThen(robot.waitToRolloutInAuton())
				.andThen(robotContainer.swerve.commands.driveForTime(
					Degrees.of(60),
					0.35,
					Degrees.of(0),
					Seconds.of(1.5)
				));
		
		return robot.waitToStartAuton().andThen(
			new SelectCommand<>(Map.ofEntries(
				Map.entry(StartPosition.AMP_SIDE, new SelectCommand<>(Map.ofEntries(
					Map.entry(Alliance.Red, blueSourceRedAmp.get()),
					Map.entry(Alliance.Blue, blueAmpRedSource.get())
				), getAlliance)),
				Map.entry(StartPosition.MIDDLE, middle),
				Map.entry(StartPosition.SOURCE_SIDE, new SelectCommand<>(Map.ofEntries(
					Map.entry(Alliance.Red, blueAmpRedSource.get()),
					Map.entry(Alliance.Blue, blueSourceRedAmp.get())
				), getAlliance))
			), StartPosition::getSelectedStartPosition)
		);
		
	}),
	
	TWO_NOTE("Two Note", robotContainer -> {
		
		RobotContainer.Commands robot = robotContainer.commands;
		Supplier<Alliance> getAlliance = () ->
			DriverStation.getAlliance().orElse(Auton.getDefaultAlliance());
		
		Supplier<Command> blueAmpRedSource = () ->
			robot.shootBelliedUpToSubwoofer()
			.andThen(robot.grabNoteAndReturn(
				Degrees.of(-60),
				0.35,
				Seconds.of(1.65)
			)).andThen(robot.shootBelliedUpToSubwoofer());
		
		Command middle =
			robot.shootBelliedUpToSubwoofer()
			.andThen(robot.grabNoteAndReturn(
				Degrees.of(0),
				0.35,
				Seconds.of(1)
			)).andThen(robot.shootBelliedUpToSubwoofer());
		
		Supplier<Command> blueSourceRedAmp = () ->
			robot.shootBelliedUpToSubwoofer()
			.andThen(robot.grabNoteAndReturn(
				Degrees.of(65),
				0.35,
				Seconds.of(1.65)
			)).andThen(robot.shootBelliedUpToSubwoofer());
		
		return new SelectCommand<>(Map.ofEntries(
			Map.entry(StartPosition.AMP_SIDE, new SelectCommand<>(Map.ofEntries(
				Map.entry(Alliance.Red, blueSourceRedAmp.get()),
				Map.entry(Alliance.Blue, blueAmpRedSource.get())
			), getAlliance)),
			Map.entry(StartPosition.MIDDLE, middle),
			Map.entry(StartPosition.SOURCE_SIDE, new SelectCommand<>(Map.ofEntries(
				Map.entry(Alliance.Red, blueAmpRedSource.get()),
				Map.entry(Alliance.Blue, blueSourceRedAmp.get())
			), getAlliance))
		), StartPosition::getSelectedStartPosition);
		
	}),
	
	THREE_NOTE("Three Note (MIDDLE POSITION ONLY)", robotContainer -> {
		
		double translationSpeed = 0.5;
		Measure<Time> diagonalDriveTime = Seconds.of(1.15);
		RobotContainer.Commands robot = robotContainer.commands;
		
		return new SelectCommand<>(Map.ofEntries(
			Map.entry(Alliance.Blue, robot.shootBelliedUpToSubwoofer()
				.andThen(robot.grabNote2FromMiddlePosition(translationSpeed, Seconds.of(0.8)))
				.andThen(robot.shootBelliedUpToSubwoofer())
				.andThen(robot.grabNote3FromMiddlePosition(translationSpeed, diagonalDriveTime))
				.andThen(robot.shootBelliedUpToSubwoofer())),
			Map.entry(Alliance.Red, robot.shootBelliedUpToSubwoofer()
				.andThen(robot.grabNote2FromMiddlePosition(translationSpeed, Seconds.of(0.8)))
				.andThen(robot.shootBelliedUpToSubwoofer())
				.andThen(robot.grabNote1FromMiddlePosition(translationSpeed, diagonalDriveTime))
				.andThen(robot.shootBelliedUpToSubwoofer()))
		), () -> DriverStation.getAlliance().orElse(Auton.getDefaultAlliance()));
		
	}),
	
	FOUR_NOTE("Four Note (MIDDLE POSITION ONLY)", robotContainer -> {
			
		double translationSpeed = 0.5;
		Measure<Time> diagonalDriveTime = Seconds.of(1.15);
		RobotContainer.Commands robot = robotContainer.commands;
		
		return new SelectCommand<>(Map.ofEntries(
			Map.entry(Alliance.Blue, robot.shootBelliedUpToSubwoofer()
				.andThen(robot.grabNote2FromMiddlePosition(translationSpeed, Seconds.of(0.8)))
				.andThen(robot.shootBelliedUpToSubwoofer())
				.andThen(robot.grabNote1FromMiddlePosition(translationSpeed, diagonalDriveTime))
				.andThen(robot.shootBelliedUpToSubwoofer())
				.andThen(robot.grabNote3FromMiddlePosition(translationSpeed, diagonalDriveTime))
				.andThen(robot.shootBelliedUpToSubwoofer())),
			Map.entry(Alliance.Red, robot.shootBelliedUpToSubwoofer()
				.andThen(robot.grabNote2FromMiddlePosition(translationSpeed, Seconds.of(0.8)))
				.andThen(robot.shootBelliedUpToSubwoofer())
				.andThen(robot.grabNote3FromMiddlePosition(translationSpeed, diagonalDriveTime))
				.andThen(robot.shootBelliedUpToSubwoofer())
				.andThen(robot.grabNote1FromMiddlePosition(translationSpeed, diagonalDriveTime))
				.andThen(robot.shootBelliedUpToSubwoofer()))
		), () -> DriverStation.getAlliance().orElse(Auton.getDefaultAlliance()));
		
	});
	
	/**
	 * The Shuffleboard widget used for selecting the auton to run.
	 */
	private static final SendableChooser<Auton> SHUFFLEBOARD_SELECTOR =
		new SendableChooser<>();
	
	/**
	 * The default Alliance color to assume if no alliance color is able to be
	 * fetched from the FMS.
	 */
	private static final Alliance DEFAULT_AUTON_ALLIANCE = Alliance.Red;
	
	/**
	 * The default auton to run if no auton is explicitly selected.
	 */
	private static final Auton DEFAULT_AUTON = Auton.FOUR_NOTE;
	
	/**
	 * A flag indicating whether or not the Shuffleboard auton selector has been
	 * initialized.
	 */
	private static boolean hasShuffleboardSelectorBeenInitialized = false;
	
	/**
	 * The human-readable name of this auton.
	 */
	private final String humanReadableName;
	
	/**
	 * The function that supplies the Command object for this auton.
	 */
	private final Function<RobotContainer, Command> commandSupplier;
	
	/**
	 * Constructs a new Auton with the given name and command supplier.
	 *
	 * @param name The human-readable name of this auton.
	 * @param commandFunction The function that supplies the Command object for
	 * this auton.
	 */
	Auton(String name, Function<RobotContainer, Command> commandFunction) {
		
		this.humanReadableName = name;
		this.commandSupplier = commandFunction;
		
	}
	
	public static Alliance getDefaultAlliance() {
		
		return Auton.DEFAULT_AUTON_ALLIANCE;
		
	}
	
	/**
	 * Initializes the Shuffleboard widget used for selecting the starting
	 * position of the robot.
	 */
	public static void initializeShuffleboardSelector() {
		
		if (Auton.hasShuffleboardSelectorBeenInitialized) return;
		else Auton.hasShuffleboardSelectorBeenInitialized = true;
		
		Auton.SHUFFLEBOARD_SELECTOR.setDefaultOption(
			Auton.DEFAULT_AUTON.getHumanReadableName(),
			Auton.DEFAULT_AUTON
		);
		
		for (Auton auton: Auton.values()) {
			
			if (auton == Auton.DEFAULT_AUTON) continue;
			
			Auton.SHUFFLEBOARD_SELECTOR.addOption(
				auton.getHumanReadableName(),
				auton
			);
			
		}
		
		Shuffleboard.getTab("Pre-match Tab").add(
			"Auton Chooser",
			Auton.SHUFFLEBOARD_SELECTOR
		);
		
	}
	
	/**
	 * Returns the auton selected by the driver from the Shuffleboard widget.
	 *
	 * @return The auton selected by the driver from the Shuffleboard widget.
	 */
	public static Auton getSelectedAuton() {
		
		return Auton.SHUFFLEBOARD_SELECTOR.getSelected();
		
	}
	
	/**
	 * Runs the auton currently selected by the Shuffleboard widget.
	 *
	 * @param robot The robot on which to run the auton.
	 */
	public static void runSelectedAuton(RobotContainer robot) {
		
		Auton.getSelectedAuton().getCommand(robot).schedule();
		
	}
	
	/**
	 * Returns the Command object for this auton.
	 *
	 * @param robot The robot on which to run the auton.
	 * @return The Command object for this auton.
	 */
	public Command getCommand(RobotContainer robot) {
		
		return this.commandSupplier.apply(robot);
		
	}
	
	/**
	 * Returns the human-readable name of this auton.
	 *
	 * @return The human-readable name of this auton.
	 */
	public String getHumanReadableName() {
		
		return this.humanReadableName;
		
	}
	
}
