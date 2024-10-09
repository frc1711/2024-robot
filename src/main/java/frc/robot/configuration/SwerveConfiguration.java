package frc.robot.configuration;

public class SwerveConfiguration {
	
	public static final SwerveModuleConfiguration FRONT_LEFT_MODULE =
		new SwerveModuleConfiguration(
			0,
			CANDevice.FRONT_LEFT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_LEFT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_LEFT_ENCODER.id,
			DoublePreference.FRONT_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
			true,
			true
		);
	
	public static final SwerveModuleConfiguration FRONT_RIGHT_MODULE =
		new SwerveModuleConfiguration(
			1,
			CANDevice.FRONT_RIGHT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_RIGHT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.FRONT_RIGHT_ENCODER.id,
			DoublePreference.FRONT_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
			true,
			false
		);
	
	public static final SwerveModuleConfiguration REAR_LEFT_MODULE =
		new SwerveModuleConfiguration(
			2,
			CANDevice.REAR_LEFT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.REAR_LEFT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.REAR_LEFT_ENCODER.id,
			DoublePreference.REAR_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
			false,
			true
			
		);
	
	public static final SwerveModuleConfiguration REAR_RIGHT_MODULE =
		new SwerveModuleConfiguration(
			3,
			CANDevice.REAR_RIGHT_STEER_MOTOR_CONTROLLER.id,
			CANDevice.REAR_RIGHT_DRIVE_MOTOR_CONTROLLER.id,
			CANDevice.REAR_RIGHT_ENCODER.id,
			DoublePreference.REAR_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
			false,
			false
		);
	
	public static final SwerveModuleConfiguration[] MODULES = {
		FRONT_LEFT_MODULE,
		FRONT_RIGHT_MODULE,
		REAR_LEFT_MODULE,
		REAR_RIGHT_MODULE
	};
	
}
