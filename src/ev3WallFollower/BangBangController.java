package ev3WallFollower;


import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController{
	private final int bandCenter, bandwidth;
	private final int motorLow = 50;
	private final int motorHigh = 150;
	private final int FILTER_OUT = 20;
	private final int offset = 20;
	private int distance;
	private int filterControl;
	private boolean isReversing;

	
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	public BangBangController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
							  int bandCenter, int bandwidth) {
		//Default Constructor
		this.bandCenter =bandCenter;
		this.bandwidth = bandwidth;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		filterControl = 0;
	}
	
	@Override
	public void processUSData(int pDistance) {
		
		//Do not alter motors when reversing
		if(isReversing)
			return;

		// rudimentary filter - toss out invalid samples corresponding to null signal.
		if(pDistance > 255)
		{
			//impossible, sensor can only read to 255, must be a bad value
		}
		else if (pDistance == 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the filter value
			filterControl ++;
		} else if (pDistance == 255){
			// true 255, therefore set distance to 255
			distance = pDistance;
		} else {
			// distance went below 255, therefore reset everything.
			filterControl = 0;
			distance = pDistance;
		}

		//calculate distance
		int distanceError = distance - bandCenter;

		// if too close to the wall, complete the reverse method 
		if(distance <10)
			reverse();
		
		//Correct distance to wall
		if(Math.abs(distanceError) <= bandwidth)
		{
			leftMotor.setSpeed(motorHigh);				// Start robot moving forward
			rightMotor.setSpeed(motorHigh);
			leftMotor.forward();
			rightMotor.forward();
		}
		//Too far from wall
		else if (distanceError > bandwidth)
		{
			//offset value is used to tweak turning radius, don't want it to sharp
			leftMotor.setSpeed(motorLow + 2*offset);				// Start robot moving forward
			rightMotor.setSpeed(motorHigh -offset);
			leftMotor.forward();
			rightMotor.forward();
		}
		//Too close to wall
		else if (distanceError < bandwidth*(-1))
		{
			//again, offset is used to tweak turning radius, we want it turn away from the wall sharply
			leftMotor.setSpeed(motorHigh +offset);				// Start robot moving forward
			rightMotor.setSpeed(motorLow -offset/2);
			leftMotor.forward();
			rightMotor.forward();
		}
		
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
	
	//method to reverse away from the wall
	@Override
	public void reverse(){
		leftMotor.stop();
		rightMotor.stop(); 
		leftMotor.setSpeed(motorHigh);
		rightMotor.setSpeed(motorHigh);
		leftMotor.rotate(-60,true);
		rightMotor.rotate(-360, false);
	}

}
