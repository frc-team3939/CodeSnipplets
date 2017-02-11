public class Robot extends IterativeRobot {	
	RobotDrive tankDrive = new RobotDrive(0, 1);
	Encoder encoder = new Encoder(0, 1, true, EncodingType.k4X);

	public static final double WHEEL_DIAMETER = 4;
	public static final double PULSE_PER_REVOLUTION = 360;
	public static final double ENCODER_GEAR_RATIO = 0;
	public static final double GEAR_RATIO = 8.45 / 1;
	public static final double FUDGE_FACTOR = 1.0;

        public void autonomousInit() {	
                final double distancePerPulse = Math.PI * Defines.WHEEL_DIAMETER / Defines.PULSE_PER_REVOLUTION
				/ Defines.ENCODER_GEAR_RATIO / Defines.GEAR_RATIO * Defines.FUDGE_FACTOR;
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