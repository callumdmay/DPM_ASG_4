package ev3Localization;

import ev3Navigator.Navigator;
import ev3Odometer.Odometer;
import lejos.robotics.SampleProvider;

public class LightLocalizer {
	
	public static int 		ROTATION_SPEED 		= 25;
	private final int 		lineDetectionValue = 40;
	private final double	light_SensorDistanceFromOrigin = 13.3;
	
	private Odometer 			odometer;
	private SampleProvider 		colorSensor;
	private float[] 			colorData;	
	private Navigator 			navigator;
	

	public LightLocalizer(Odometer odo, Navigator navigator, SampleProvider colorSensor, float[] colorData) {
		this.odometer = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.navigator = navigator;
	}

	public void doLocalization() {
		// drive to location listed in tutorial
		// start rotating and clock all 4 gridlines
		// do trig to compute (0,0) and 0 degrees
		// when done travel to (0,0) and turn to 0 degrees


		double blackLineAngles[] = new double[4];
		navigator.travelTo(-5, -5);
		navigator.turnTo(Math.PI/2);

		for( int index = 0 ; index < blackLineAngles.length; index ++)
		{
			
			//Capture the angle when we first encounter the black line
			while(!blackLineDetected())
				navigator.rotateCounterClockWise(ROTATION_SPEED);
			
			double angle1 = odometer.getTheta();
			
			//Capture the angle when we leave the black line
			while(blackLineDetected())
				navigator.rotateCounterClockWise(ROTATION_SPEED);
			
			double angle2 = odometer.getTheta();
		
			//Return the average of the two angles - when we first detect the black line, and when we stop detecting it
			blackLineAngles[index]= calculateAngleAverage(angle1, angle2);
		}
		
		double deltaY = blackLineAngles[2] - blackLineAngles[0];
		double deltaX = blackLineAngles[3] - blackLineAngles[1];

		
		odometer.setX(-light_SensorDistanceFromOrigin * Math.cos(deltaY/2));

		odometer.setY(-light_SensorDistanceFromOrigin * Math.cos(deltaX/2));
	}

	private boolean blackLineDetected()
	{
		colorSensor.fetchSample(colorData, 0);

		//if we run over a black line, calculate and update odometer values
		if((int)(colorData[0]*100) < lineDetectionValue)
			return true;
		else 
			return false;
	}
	
	private double calculateAngleAverage(double angle1, double angle2)
	{
		double x = Math.abs(angle1 -angle2);

		if (x < Math.PI) 
			return  ((angle1 + angle2) / 2) % (2 *Math.PI);
		if (x != Math.PI)
			return (((angle1 + angle2) / 2) + Math.PI) % (2 *Math.PI);

		throw new ArithmeticException("Could not calculate angle average of numbers");

	}


}
