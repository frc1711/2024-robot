package frc.robot.configuration;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;

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
	
	public static final Measure<Distance> SWERVE_MODULE_WHEEL_DIAMETER = Inches.of(4);
	
	public static final double SWERVE_MODULE_DRIVE_MOTOR_RATIO =
		(50f/14f) * (16f/28f) * (45f/15f);
	
	public static final Measure<Per<Distance, Angle>> SWERVE_MODULE_DRIVE_MOTOR_DISTANCE_PER_REVOLUTION =
		(SWERVE_MODULE_WHEEL_DIAMETER.times(Math.PI)).divide(SWERVE_MODULE_DRIVE_MOTOR_RATIO).per(Rotation);
	
	public static final double SWERVE_MODULE_STEERING_MOTOR_RATIO = (1f/12.8f);
	
}
