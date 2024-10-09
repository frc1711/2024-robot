package frc.robot.devicewrappers;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Angle;

import static edu.wpi.first.units.Units.Degrees;

public class RaptorsNavX {
	
	protected final AHRS navX;
	
	protected Measure<Angle> adjustmentAngle;
	
	public RaptorsNavX() {
		
		this.navX = new AHRS();
		this.adjustmentAngle = Degrees.of(0);
		
	}
	
	public Measure<Angle> getRotation() {
		
		Measure<Angle> result = Degrees.of(
			this.navX.getRotation2d().getDegrees()
		);
		
		result = result.minus(this.adjustmentAngle);
		
		return result;
		
	}
	
	public void calibrate() {
	
		this.calibrate(Degrees.of(0));
	
	}
	
	public void calibrate(Measure<Angle> currentHeading) {
		
		this.navX.reset();
		this.adjustmentAngle = currentHeading;
		
	}
	
}
