package ev3Navigator;



/*
 * 
 * This class contains utility/math methods that are used by the navigator class
 * These methods were refactored out of the Navigator class to reduce the size of the
 * navigator class and increase readability
 * These methods are only dependent on their parameters, hence why they are static.
 * 
 */
public class NavigatorUtility {



	//This method calculates the new angle the robot must face, based on the delta Y and delta X
	public static double calculateNewAngle(double deltaX, double deltaY, double currentAngle)
	{

		if(deltaX >= 0 )
			return calculateAngleError(Math.atan(deltaY/deltaX), currentAngle);

		if(deltaX< 0 && deltaY >= 0)
			return calculateAngleError(Math.atan(deltaY/deltaX)+ Math.PI, currentAngle);

		if(deltaX < 0 && deltaY < 0)
			return calculateAngleError(Math.atan(deltaY/deltaX) - Math.PI,currentAngle);

		throw new ArithmeticException("Cannot calculate new angle");
	}

	//This method determines how much the robot should turn, it return the smallest turning angle possible
	private static double calculateAngleError(double newAngle, double currentAngle)
	{
		double deltaTheta = newAngle - currentAngle;

		if( Math.abs(deltaTheta) <= Math.PI)
			return deltaTheta;

		if(deltaTheta < -Math.PI)
			return deltaTheta + 2*Math.PI;

		if(deltaTheta > Math.PI)
			return deltaTheta - 2*Math.PI;

		throw new ArithmeticException("Cannot calculate angle error");

	}

	//Convert radian angle we want into an angle the motor can turn to
	public static int convertAngle(double radius, double width, double angle) {
		return (int) ((180.0 * Math.PI * width * angle / 360.0) / (Math.PI * radius));
	}
}
