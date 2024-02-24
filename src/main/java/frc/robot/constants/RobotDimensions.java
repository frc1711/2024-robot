package frc.robot.constants;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.units.Units.*;

/**
 * A collections of constants representing the various important dimensions of
 * the robot.
 */
public final class RobotDimensions {
	
	/**
	 * The total width (left to right) of the chassis of the robot.
	 */
	public static final Measure<Distance> ROBOT_WIDTH = Inches.of(30);
	
	/**
	 * The total length (front to back) of the chassis of the robot.
	 */
	public static final Measure<Distance> ROBOT_LENGTH = Inches.of(28);
	
	/**
	 * The center distance between the front and rear swerve modules.
	 */
	public static final Measure<Distance> LENGTHWISE_WHEELBASE = Inches.of(20);
	
	/**
	 * The center distance between the left and right swerve modules.
	 */
	public static final Measure<Distance> WIDTHWISE_WHEELBASE = Inches.of(21.25);
	
}
