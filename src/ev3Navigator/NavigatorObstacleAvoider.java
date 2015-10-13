package ev3Navigator;

import ev3Odometer.Odometer;
import ev3WallFollower.UltrasonicController;
import ev3WallFollower.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class NavigatorObstacleAvoider {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor neckMotor;

	private double wheelRadius;
	private double axleLength;

	private Odometer odometer;
	private UltrasonicPoller ultraSonicPoller;
	private UltrasonicController wallFollowerController;

	private final double obstacleDistance = 20;
	private final double wallFollowingAngleError = 4 ;

	private boolean isNavigating = false;


	private boolean isAvoiding = false;

	private final int neckMotor_OFFSET = 60;
	private final int FILTER_OUT = 5;
	private int filterControl;

	public NavigatorObstacleAvoider(Odometer pOdometer, UltrasonicPoller pUltraSonicPoller, UltrasonicController pwallFollowerController, EV3LargeRegulatedMotor pLeftMotor, EV3LargeRegulatedMotor pRightMotor, 
			EV3LargeRegulatedMotor pNeckMotor, double pWheelRadius, double pAxleLength)
	{
		ultraSonicPoller 			= pUltraSonicPoller;
		wallFollowerController 		= pwallFollowerController;
		odometer 					= pOdometer;
		leftMotor 					= pLeftMotor;
		rightMotor 					= pRightMotor;
		neckMotor 					= pNeckMotor;
		wheelRadius 				= pWheelRadius;
		axleLength 					= pAxleLength;
	}


	//This method checks for obstacles in front of the robot as it is moving forward
	public void checkForObstacles()
	{
		if(ultraSonicPoller == null)
			return;

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
		if(neckMotor == null)
			return;

		isNavigating = false;
		isAvoiding = true;

		rightMotor.stop();
		leftMotor.stop();

		neckMotor.rotate(-1 * neckMotor_OFFSET, true);
		leftMotor.rotate(NavigatorUtility.convertAngle(wheelRadius, axleLength, 90), true);
		rightMotor.rotate(-NavigatorUtility.convertAngle(wheelRadius, axleLength, 90), false);
	}

	/*
	 * This method basically runs the p-type wall-following algorithm
	 *until the robot is facing back at towards coordinates. It then tries to move towards them again
	 */

	public void avoidObstacle(double pX, double pY) {

		if(ultraSonicPoller == null || wallFollowerController == null)
			return;

		double currentX;
		double currentY;

		do{
			currentX = odometer.getX();
			currentY = odometer.getY();
			wallFollowerController.processUSData(ultraSonicPoller.getDistance());
		} while(Math.abs(NavigatorUtility.calculateAngleError(pX - currentX, pY - currentY, odometer.getTheta())*180/Math.PI) > wallFollowingAngleError);

		returnToNavigation();
	}

	//Reset the neck motor to face forward and continue towards coordinates
	private void  returnToNavigation()
	{
		if(neckMotor == null)
			return;

		isNavigating = true;
		isAvoiding = false;
		neckMotor.rotate(neckMotor_OFFSET, false);
	}

	public boolean isNavigating() {
		return isNavigating;
	}

	public void setNavigating(boolean isNavigating) {
		this.isNavigating = isNavigating;
	}
	
	public boolean isAvoiding() {
		return isAvoiding;
	}

	public void setAvoiding(boolean isAvoiding) {
		this.isAvoiding = isAvoiding;
	}



}
