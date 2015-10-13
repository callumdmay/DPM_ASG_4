package ev3Navigator;

import java.util.Queue;

import ev3Odometer.Odometer;
import ev3WallFollower.UltrasonicController;
import ev3WallFollower.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigator extends Thread{

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor neckMotor;

	private double wheelRadius;
	private double axleLength;

	private final double locationError = 1;
	private final double navigatingAngleError = 1;

	private int FORWARD_SPEED = 250;
	private int ROTATE_SPEED = 150;

	private Odometer odometer;

	private NavigatorObstacleAvoider obstacleAvoider;

	public static int coordinateCount = 0;
	private static Queue<Coordinate> coordinates;


	public Navigator(Odometer pOdometer, UltrasonicPoller pUltraSonicPoller, UltrasonicController pwallFollowerController, EV3LargeRegulatedMotor pLeftMotor, EV3LargeRegulatedMotor pRightMotor, 
			EV3LargeRegulatedMotor pNeckMotor, double pWheelRadius, double pAxleLength)
	{
		odometer 					= pOdometer;
		leftMotor 					= pLeftMotor;
		rightMotor 					= pRightMotor;
		neckMotor 					= pNeckMotor;
		wheelRadius 				= pWheelRadius;
		axleLength 					= pAxleLength;

		obstacleAvoider = new NavigatorObstacleAvoider(pOdometer, pUltraSonicPoller, pwallFollowerController, pLeftMotor, 
				pRightMotor, pNeckMotor, pWheelRadius,pAxleLength );
	}

	public Navigator(Odometer pOdometer, EV3LargeRegulatedMotor pLeftMotor, EV3LargeRegulatedMotor pRightMotor, double pWheelRadius, double pAxleLength)
	{
		odometer 					= pOdometer;
		leftMotor 					= pLeftMotor;
		rightMotor 					= pRightMotor;
		neckMotor 					= null;
		wheelRadius 				= pWheelRadius;
		axleLength 					= pAxleLength;
	}

	@Override
	public void run()
	{
		resetMotors();

		//For each coordinate in the queue, 
		for( Coordinate coordinate : coordinates)
		{
			travelTo(coordinate.getX(), coordinate.getY());

			//wait for robot to finish navigating too coordinate before moving to next one
			while(obstacleAvoider.isNavigating() ||obstacleAvoider.isAvoiding()){}

			coordinateCount++;
		}
		resetMotors();
	}

	//This method takes a new x and y location, and moves to it while avoiding obstacles
	public void travelTo(double pX, double pY)
	{

		obstacleAvoider.setNavigating(true);

		//While the robot is not at the objective coordinates, keep moving towards it 
		while(Math.abs(pX- odometer.getX()) > locationError || Math.abs(pY - odometer.getY()) > locationError)
		{
			obstacleAvoider.checkForObstacles();

			if(obstacleAvoider.isNavigating())
				navigateToCoordinates(pX, pY);

			if(obstacleAvoider.isAvoiding())
				obstacleAvoider.avoidObstacle(pX, pY);
		}

		obstacleAvoider.setNavigating(false);

	}

	//Turns to the absolute value theta
	public void turnTo(double pTheta)
	{

		pTheta = pTheta % Math.toRadians(360);

		double deltaTheta = pTheta - odometer.getTheta();

		double rotationAngle = 0;

		if( Math.abs(deltaTheta) <= Math.PI)
			rotationAngle = deltaTheta;

		if(deltaTheta < -Math.PI)
			rotationAngle = deltaTheta + 2*Math.PI;

		if(deltaTheta > 2*Math.PI)
			rotationAngle = deltaTheta - 2*Math.PI;


		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		leftMotor.rotate(-NavigatorUtility.convertAngle(wheelRadius, axleLength, rotationAngle * 180/Math.PI), true);
		rightMotor.rotate(NavigatorUtility.convertAngle(wheelRadius, axleLength, rotationAngle * 180/Math.PI), false);
	}

	public void turnTo(double pTheta, int speed)
	{

		pTheta = pTheta % Math.toRadians(360);

		double deltaTheta = pTheta - odometer.getTheta();

		double rotationAngle = 0;

		if( Math.abs(deltaTheta) <= Math.PI)
			rotationAngle = deltaTheta;

		if(deltaTheta < -Math.PI)
			rotationAngle = deltaTheta + 2*Math.PI;

		if(deltaTheta > Math.PI)
			rotationAngle = deltaTheta - 2*Math.PI;


		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);

		leftMotor.rotate(-NavigatorUtility.convertAngle(wheelRadius, axleLength, rotationAngle * 180/Math.PI), true);
		rightMotor.rotate(NavigatorUtility.convertAngle(wheelRadius, axleLength, rotationAngle * 180/Math.PI), false);
	}


	/*
	 * This method simply navigates to the given coordinates
	 */
	private void navigateToCoordinates(double pX, double pY)
	{
		double currentX = odometer.getX();
		double currentY = odometer.getY();

		double newAngle = NavigatorUtility.calculateNewAngle(pX - currentX, pY - currentY);


		if(Math.abs(NavigatorUtility.calculateShortestTurningAngle(newAngle, odometer.getTheta())*180/Math.PI)> navigatingAngleError)
			turnTo(newAngle);
		else
		{
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.forward();
			rightMotor.forward();
		}
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

		if(neckMotor == null)
		{		
			neckMotor.stop();
			neckMotor.setAcceleration(2000);
			neckMotor.setSpeed(100);
		}
	}


	public void stopMotors()
	{
		leftMotor.stop();
		rightMotor.stop();
	}
	public void rotateClockWise(int speed)
	{
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);

		leftMotor.forward();
		rightMotor.backward();
	}

	public void rotateCounterClockWise(int speed)
	{
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);

		leftMotor.backward();
		rightMotor.forward();
	}

	public void setFORWARD_SPEED(int fORWARD_SPEED) {
		FORWARD_SPEED = fORWARD_SPEED;
	}

	public void setROTATE_SPEED(int rOTATE_SPEED) {
		ROTATE_SPEED = rOTATE_SPEED;
	}

}

