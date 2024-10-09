package frc.robot.configuration;

public class SwerveModuleConfiguration {
	
	public final int moduleID;
	
	public final int steerMotorControllerCANID;
	
	public final int driveMotorControllerCANID;
	
	public final int encoderCANID;
	
	public final DoublePreference encoderOffset;
	
	public final boolean isFront;
	
	public final boolean isLeft;
	
	public SwerveModuleConfiguration(
		int moduleID,
		int steerMotorControllerCANID,
		int driveMotorControllerCANID,
		int encoderCANID,
		DoublePreference encoderOffset,
		boolean isFront,
		boolean isLeft
	) {
		
		this.moduleID = moduleID;
		this.steerMotorControllerCANID = steerMotorControllerCANID;
		this.driveMotorControllerCANID = driveMotorControllerCANID;
		this.encoderCANID = encoderCANID;
		this.encoderOffset = encoderOffset;
		this.isFront = isFront;
		this.isLeft = isLeft;
		
	}
	
}
