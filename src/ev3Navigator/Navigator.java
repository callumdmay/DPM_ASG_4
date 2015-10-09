package ev3Navigator;

import java.util.Queue;

import ev3Odometer.Odometer;
import ev3WallFollower.PController;
import ev3WallFollower.UltrasonicController;
import ev3WallFollower.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigator extends Thread{

	private boolean isNavigating = false;
	private boolean isAvoiding = false;

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor neckMotor;

	private double wheelRadius;
	private double axleLength;
	
	private final double obstacleDistance = 20;

	private final double locationError = 1;
	private final double navigatingAngleError = 1;
	private final double wallFollowingAngleError = 4 ;
	
	private final int neckMotor_OFFSET = 60;
	private final int FILTER_OUT = 5;
	private int filterControl;

	private final int FORWARD_SPEED = 250;
	private final int ROTATE_SPEED = 150;

	private Odometer odometer;
	private UltrasonicPoller ultraSonicPoller;
	private UltrasonicController pController;
	
	public static int coordinateCount = 0;
	private static Queue<Coordinate> coordinates;

	
	public Navigator(Odometer pOdometer, UltrasonicPoller pUltraSonicPoller, UltrasonicController p_controller, EV3LargeRegulatedMotor pLeftMotor, EV3LargeRegulatedMotor pRightMotor, 
			EV3LargeRegulatedMotor pNeckMotor, double pWheelRadius, double pAxleLength)
	{
		ultraSonicPoller 	= pUltraSonicPoller;
		pController 		= p_controller;
		odometer 			= pOdometer;
		leftMotor 			= pLeftMotor;
		rightMotor 			= pRightMotor;
		neckMotor 			= pNeckMotor;
		wheelRadius 		= pWheelRadius;
		axleLength 			= pAxleLength;
	}

	public void run()
	{
		resetMotors();
		
		//For each coordinate in the queue, 
		for( Coordinate coordinate : coordinates)
		{
			travelTo(coordinate.getX(), coordinate.getY());
			
			//wait for robot to finish navigating too coordinate before moving to next one
			while(isNavigating ||isAvoiding){}
			coordinateCount++;
		}
		resetMotors();
	}



	//This method takes a new x and y location, and moves to it while avoiding obstacles
	public void travelTo(double pX, double pY)
	{

		isNavigating = true;
		
		//While the robot is not at the objective coordinates, keep moving towards it 
		while(Math.abs(pX- odometer.getX()) > locationError || Math.abs(pY - odometer.getY()) > locationError)
		{
			checkForObstacles();
			
			if(isNavigating)
				navigateToCoordinates(pX, pY);
			if(isAvoiding)
				avoidObstacle(pX, pY);
		}
		
		isNavigating = false;

	}

	//Takes a relative angle theta, and turns that many radians left or right, depending on sign of the angle
	public void turnTo(double pTheta)
	{
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		leftMotor.rotate(-NavigatorUtility.convertAngle(wheelRadius, axleLength, pTheta * 180/Math.PI), true);
		rightMotor.rotate(NavigatorUtility.convertAngle(wheelRadius, axleLength, pTheta * 180/Math.PI), false);
	}

	
	//This method checks for obstacles in front of the robot as it is moving forward
	private void checkForObstacles()
	{
		int pDistance = ultraSonicPoller.getDistance();
		// rudimentary filter - checks 5 times to ensure obstacle is really ahead of robot
		if(pDistance < obstacleDistance)
		{
			filterControl ++;
		}

		//We must get 5 readings of less than 25 before we initiate obstacle avoidance
		if(filterControl < FILTER_OUT)
			return;

		filterControl = 0;

		initiateObstacleAvoidance();

	}

	//Change from navigating to avoiding obstacles
	private void initiateObstacleAvoidance()
	{
		isNavigating = false;
		isAvoiding = true;
		
		rightMotor.stop();
		leftMotor.stop();
		
		neckMotor.rotate(-1 * neckMotor_OFFSET, true);
		leftMotor.rotate(NavigatorUtility.convertAngle(wheelRadius, axleLength, 90), true);
		rightMotor.rotate(-NavigatorUtility.convertAngle(wheelRadius, axleLength, 90), false);
	}
	
	//Reset the neck motor to face forward and continue towards coordinates
	private void  returnToNavigation()
	{
		isNavigating = true;
		isAvoiding = false;
		neckMotor.rotate(neckMotor_OFFSET, false);
	}

	/*
	 * This method simply navigates to the given coordinates
	 * 
	 */
	private void navigateToCoordinates(double pX, double pY)
	{
		double currentX = odometer.getX();
		double currentY = odometer.getY();

		double newAngle = NavigatorUtility.calculateNewAngle(pX - currentX, pY - currentY, odometer.getTheta());

		if(Math.abs(newAngle*180/Math.PI)> navigatingAngleError)
			turnTo(newAngle);
		else
		{
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.forward();
			rightMotor.forward();
		}
	}
	
	/*
	 * This method basically runs the p-type wall-following algorithm
	 *until the robot is facing back at towards coordinates. It then tries to move towards them again
	*/
	private void avoidObstacle(double pX, double pY) {
		double currentX;
		double currentY;
		
		do{
			currentX = odometer.getX();
			currentY = odometer.getY();
			pController.processUSData(ultraSonicPoller.getDistance());
		} while(Math.abs(NavigatorUtility.calculateNewAngle(pX - currentX, pY - currentY, odometer.getTheta())*180/Math.PI) > wallFollowingAngleError);
		
		returnToNavigation();
	}
	

	//Sets the global coordinates for the navigator
	public void setCoordinates(Queue<Coordinate> pCoordinates)
	{
		coordinates = pCoordinates;
	}
	
	//Stops and resets the motors
	private void resetMotors()
	{
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(2000);
		}
		
		neckMotor.stop();
		neckMotor.setAcceleration(2000);
		neckMotor.setSpeed(100);
	}
	

	public boolean isNavigating() {
		return isNavigating;

	}


}

