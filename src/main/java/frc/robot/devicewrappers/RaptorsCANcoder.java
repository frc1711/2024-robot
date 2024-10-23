package frc.robot.devicewrappers;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Unit;
import frc.robot.constants.DoublePreference;

public class RaptorsCANcoder /*extends RaptorsEncoder*/ {
	
	protected final CANcoder encoder;
	
	public RaptorsCANcoder(
		CANcoder encoder,
		DoublePreference zeroOffsetPreference,
		Unit<Angle> zeroOffsetPreferenceUnits
	) {
		
//		super(zeroOffsetPreference, zeroOffsetPreferenceUnits);
		
		this.encoder = encoder;
		
	}
	
	
	
	
	
}
