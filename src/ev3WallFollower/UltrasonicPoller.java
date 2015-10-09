package ev3WallFollower;

import ev3Navigator.Navigator;
import lejos.robotics.SampleProvider;

//
//  Control of the wall follower is applied periodically by the 
//  UltrasonicPoller thread.  The while loop at the bottom executes
//  in a loop.  Assuming that the us.fetchSample, and cont.processUSData
//  methods operate in about 20mS, and that the thread sleeps for
//  50 mS at the end of each loop, then one cycle through the loop
//  is approximately 70 mS.  This corresponds to a sampling rate
//  of 1/70mS or about 14 Hz.
//


public class UltrasonicPoller extends Thread{
	private SampleProvider ultraSonicSensorSampleProvider;
	private float[] usData;
	private boolean isAvoidingObstacle;
	//set to 30 as we need the distance variable to be initialized above the obstacle detection distance
	private int distance = 30;
	private Object lock;

	public UltrasonicPoller(SampleProvider us, float[] usData) {
		this.ultraSonicSensorSampleProvider = us;
		this.usData = usData;
		isAvoidingObstacle = false;
		lock = new Object();
	}

	//  Sensors now return floats using a uniform protocol.
	//  Need to convert US result to an integer [0,255]

	public void run() {
		while (true) {
			ultraSonicSensorSampleProvider.fetchSample(usData,0);							// acquire data

			//The poller now simply updates the distance variable, it does not influence the controller at all
			synchronized(lock){
				setDistance((int)(usData[0]*100.0));
			}
			
			try { Thread.sleep(50); } catch(Exception e){}		// Poor man's timed sampling
		}
	}


	private void setDistance(int distance) {
		synchronized (lock) {
			this.distance = distance;
		}
	}


	public int getDistance() {
		int result;
		synchronized (lock) {
			result = distance;
		}

		return result;
	}



}
