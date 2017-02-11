package org.usfirst.frc.team3939.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;;

public class Drivetoadistance extends IterativeRobot {	
	RobotDrive tankDrive = new RobotDrive(0, 1);
	Encoder encoder = new Encoder(0, 1, true, EncodingType.k4X);

	public static final double WHEEL_DIAMETER = 4; //Will need to be set before use
	public static final double PULSE_PER_REVOLUTION = 360;
	public static final double ENCODER_GEAR_RATIO = 0;
	public static final double GEAR_RATIO = 8.45 / 1;
	public static final double FUDGE_FACTOR = 1.0;

        public void autonomousInit() {	
                final double distancePerPulse = Math.PI * WHEEL_DIAMETER / PULSE_PER_REVOLUTION
				/ ENCODER_GEAR_RATIO / GEAR_RATIO * FUDGE_FACTOR;
	        encoder.setDistancePerPulse(distancePerPulse);

	}

	public void autonomousPeriodic() {
	        double encoderDistanceReading = encoder.getDistance();
		SmartDashboard.putNumber("encoder reading", encoderDistanceReading);
		
		tankDrive.drive(-0.25, 0);
		if (encoderDistanceReading > 36) {
			tankDrive.drive(0, 0);
		}
	}
}