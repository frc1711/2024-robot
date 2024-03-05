package frc.robot.devicewrappers;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.*;

public class RaptorsSparkMaxNEO {
	
	public final CANSparkMax sparkMax;
	
	public RaptorsSparkMaxNEO(int canID) {
		
		this.sparkMax = new CANSparkMax(
			canID,
			CANSparkLowLevel.MotorType.kBrushless
		);
		
		this.sparkMax.restoreFactoryDefaults();
		
	}
	
	public RaptorsSparkMaxNEO inCoastMode() {
		
		this.sparkMax.setIdleMode(CANSparkBase.IdleMode.kCoast);
		
		return this;
		
	}
	
	public RaptorsSparkMaxNEO inBrakeMode() {
		
		this.sparkMax.setIdleMode(CANSparkBase.IdleMode.kBrake);
		
		return this;
		
	}
	
	public RaptorsSparkMaxNEO withInvertedOutput() {
		
		this.sparkMax.setInverted(true);
		
		return this;
		
	}
	
	public RaptorsSparkMaxNEO withoutInvertedOutput() {
		
		this.sparkMax.setInverted(false);
		
		return this;
		
	}
	
	public RaptorsSparkMaxNEO withSmartCurrentLimit(
		Measure<Current> smartCurrentLimit
	) {
		
		this.sparkMax.setSmartCurrentLimit(
			(int) Math.floor(smartCurrentLimit.in(Amps))
		);
		
		return this;
		
	}
	
	public RelativeEncoder getEncoder() {
		
		return this.sparkMax.getEncoder();
		
	}
	
	public SparkPIDController getPIDController() {
		
		return this.sparkMax.getPIDController();
		
	}
	
	public void set(double speed) {
		
		this.sparkMax.set(speed);
		
	}
	
	public void setVoltage(Measure<Voltage> voltage) {
		
		this.sparkMax.setVoltage(voltage.in(Volts));
		
	}
	
	public void stopMotor() {
		
		this.sparkMax.stopMotor();
		
	}
	
}
