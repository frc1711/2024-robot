package frc.robot.devicewrappers.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import frc.robot.configuration.*;

import java.util.stream.Stream;

public class RaptorsSwerve {
	
	public static final int FRONT_LEFT_MODULE_ID = 0;
	
	public static final int FRONT_RIGHT_MODULE_ID = 1;
	
	public static final int REAR_LEFT_MODULE_ID = 2;
	
	public static final int REAR_RIGHT_MODULE_ID = 3;
	
	protected final RaptorsSwerveModule[] modules;
	
	public RaptorsSwerve() {
		
		this.modules = new RaptorsSwerveModule[4];
		
		double swerveModuleXOffsetFromCenterInMeters =
			RobotDimensions.LENGTHWISE_WHEELBASE.in(Units.Meters) / 2;
		
		double swerveModuleYOffsetFromCenterInMeters =
			RobotDimensions.WIDTHWISE_WHEELBASE.in(Units.Meters) / 2;
		
		for (SwerveModuleConfiguration config: SwerveConfiguration.MODULES) {
			
			this.modules[config.moduleID] = new RaptorsSwerveModule(
				config.steerMotorControllerCANID,
				config.driveMotorControllerCANID,
				config.encoderCANID,
				config.encoderOffset,
				new Translation2d(
					swerveModuleXOffsetFromCenterInMeters * (config.isFront ? 1 : -1),
					swerveModuleYOffsetFromCenterInMeters * (config.isLeft ? 1 : -1)
				)
			);
			
		}
		
	}
	
	protected Stream<RaptorsSwerveModule> getModuleStream() {
		
		return Stream.of(this.modules);
		
	}
	
}
