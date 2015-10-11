package ev3Localization;



import java.util.Arrays;

import ev3Navigator.Navigator;
import ev3Odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;

public class USLocalizer {
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	
	
	public static int ROTATION_SPEED = 30;
	private final int distanceNoiseMargin = 1;
	private final int measuredDistance = 30;
	private final int usSensorMaxDistance = 40;
	
	private Odometer odometer;
	private SampleProvider usSensor;
	private float[] usData;
	private LocalizationType locType;
	private Navigator navigator;

	



	public USLocalizer(Odometer odo,  SampleProvider usSensor, float[] usData, LocalizationType locType, Navigator navigator) {
		this.odometer = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locType = locType;
		this.navigator = navigator;
	}

	public void doLocalization() {
		double [] pos = new double [3];
		double angleA, angleB;
		
		odometer.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});


		if (locType == LocalizationType.FALLING_EDGE) {


			if(getFilteredData() <= usSensorMaxDistance-3){
			// rotate the robot until it sees no wall
			while(getFilteredData() <= usSensorMaxDistance-3)
				navigator.rotateClockWise(ROTATION_SPEED);
			
			
			//quickly turn away from wall, to ensure the same value isn't captured
			navigator.turnTo(odometer.getTheta() - 30*Math.PI/180);
			}
			
			// keep rotating until the robot sees a wall, then latch the angle
			angleA = latchFallingEdgeAngle(true);

			Sound.beep();

			// switch direction and wait until it sees no wall
			while(getFilteredData() <= usSensorMaxDistance-3)
				navigator.rotateCounterClockWise(ROTATION_SPEED);

			
			// keep rotating until the robot sees a wall, then latch the angle
			angleB = latchFallingEdgeAngle(false);

			Sound.beep();
			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'

			double newTheta = odometer.getTheta() + calculateOdometerAngleAdjustment(angleA, angleB);

			// update the odometer position (example to follow:)
			odometer.setPosition(new double [] {0.0, 0.0, newTheta}, new boolean [] {true, true, true});

			navigator.turnTo(0);

		} else {
			
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall.
			 * This is very similar to the FALLING_EDGE routine, but the robot
			 * will face toward the wall for most of it.
			 */
			

			
			while(getFilteredData() >= measuredDistance - 10)
				navigator.rotateCounterClockWise(ROTATION_SPEED+20);
			
			
			while(getFilteredData() <=measuredDistance)
				navigator.rotateClockWise(ROTATION_SPEED-10);
			
			angleA = odometer.getTheta();
			Sound.beep();
			
			while(getFilteredData() >= measuredDistance - 10)
				navigator.rotateCounterClockWise(ROTATION_SPEED+20);
			
			while(getFilteredData() <=measuredDistance)
				navigator.rotateCounterClockWise(ROTATION_SPEED-10);
			
			angleB = odometer.getTheta();
			Sound.beep();
			
			double newTheta = odometer.getTheta() + calculateOdometerAngleAdjustment(angleA, angleB);

			// update the odometer position (example to follow:)
			odometer.setPosition(new double [] {0.0, 0.0, newTheta}, new boolean [] {true, true, true});

			navigator.turnTo(0);
		}
	}


	private double latchFallingEdgeAngle(boolean clockWise)
	{
		double fallingEdgeAngle1;
		double fallingEdgeAngle2;
		if(clockWise){
			// keep rotating until the robot sees a wall, then latch the angle
			while(getFilteredData()> measuredDistance+distanceNoiseMargin)
			{
				navigator.rotateClockWise(ROTATION_SPEED);
			}
			fallingEdgeAngle1 = odometer.getTheta();

			while(getFilteredData()> measuredDistance-distanceNoiseMargin)
			{
				navigator.rotateClockWise(ROTATION_SPEED);
			}

			fallingEdgeAngle2 = odometer.getTheta();

			return calculateAngleAverage(fallingEdgeAngle1, fallingEdgeAngle2);

		}

		else{

			// keep rotating until the robot sees a wall, then latch the angle
			while(getFilteredData()> measuredDistance+distanceNoiseMargin)
			{
				navigator.rotateCounterClockWise(ROTATION_SPEED);
			}
			fallingEdgeAngle1 = odometer.getTheta();

			while(getFilteredData()> measuredDistance-distanceNoiseMargin)
			{
				navigator.rotateCounterClockWise(ROTATION_SPEED);
			}

			fallingEdgeAngle2 = odometer.getTheta();

			return calculateAngleAverage(fallingEdgeAngle1, fallingEdgeAngle2);

		}
	}
	
	private double latchRisingEdgeAngle(boolean clockWise)
	{
		double risingEdgeAngle1;
		double risingEdgeAngle2;
		
		if(clockWise)
		{
			
		}
		else
		{
			
		}
		
		return 0;
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

	private double calculateOdometerAngleAdjustment(double angleA, double angleB)
	{
		if(angleA <= angleB)
			return Math.PI/4 - (angleA - angleB)/2;
		if(angleA > angleB)
			return 5*Math.PI/4 - (angleA - angleB)/2;

		throw new ArithmeticException("Could not calculate odometer angle adjustmnet");

	}


	private float getFilteredData() {
	/*
		float sampleData[] = new float[5];
		
		for(int index = 0 ; index < sampleData.length; index++)
		{
			usSensor.fetchSample(usData, 0);
			
			if(usData[0]> usSensorMaxDistance)
				usData[0] = usSensorMaxDistance;
			
			sampleData[index] = usData[0]*100;
			try {
				Thread.sleep(5);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
		Arrays.sort(sampleData);
		*/
		
		usSensor.fetchSample(usData, 0);
		
		if(usData[0]> usSensorMaxDistance)
			usData[0] = usSensorMaxDistance;
		
		float distance =  usData[0]*100;
		
		if(distance > usSensorMaxDistance)
			distance = usSensorMaxDistance;
		
		//LCD.drawString("Distance: "+sampleData[(int) Math.floor(sampleData.length/2)], 0, 4);
		//return sampleData[(int) Math.floor(sampleData.length/2)];
		LCD.drawString("Distance: "+distance, 0, 4);
		return distance;
	}
	


}
