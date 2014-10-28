package it.treisoft.quadrotor;

import java.util.ArrayList;

import android.content.Context;
import android.os.SystemClock;
import android.util.Log;
import it.treisoft.quadrotor.TcpClient.TcpMessageReceiver;
import it.treisoft.quadrotor.imuReading.Attitude;
import it.treisoft.quadrotor.imuReading.ImuThread;

public class Controller implements TcpMessageReceiver{
	
	public static final int MAX_MOTOR_POWER = 400; // 500 normally, less for testing.
	public static final double R2DG=180.0f/Math.PI;
	public static final int STATE_SEND_DIVIDER = 20;
    public static final float MAX_TIME_WITHOUT_PC_RX = 1.0f; // Maximum time [s] without any message from the PC, before emergency stop.
    public static final float MAX_TIME_WITHOUT_ADK_RX = 1.0f; // Maximum time [s] without any message from the ADK, setting the temperature to 0 (error).
    public static final long INT_MAX = 2147483648L;
    public static final float MAX_SAFE_PITCH_ROLL = 30; // [deg].
	
	
    
    public Controller(Context context)
    {
    	newMotorValue=false;	
    	
    	//initialize working structures
    	motors=new Motors();
    	setPoint=new SetPoint();
    	imu=new imuReading(context);
    	output=new Output();
    	
    	//initialize setpoint variables
    	setPoint.pitch=0;
    	setPoint.roll=0;
    	setPoint.yaw=0;
    	setPoint.altitude=0.0f;
    	    	
    	//lorenzo's quadrotor
//    	kp=0.85f; 
//    	ki=0.08f;
//    	kd=0.2f;
//    	LP=0.5f;
    	
    	//aslatech's quadrotor
    	kp=0.62f;
    	ki=0.04f;
    	kd=0.3f;
    	LP=0.5f;
    	
    	//initialize SISO controllers
    	pitchPID=new PIDRegulator(kp,ki,kd,LP);
    	rollPID=new PIDRegulator(kp,ki,kd,LP);
    	//needed more tuning
    	yawPID=new PIDRegulator(0.6f,0.01f,0.3f,LP);
    	altitudePID=new PIDRegulator(5.0f,0.03f,0.1f,LP);
    	
    	//initialize controller thread 
    	controllerThread=new ControllerThread();
    	
    	stateSendDividerCounter = 0;
    	
        timeWithoutPcRx = 0.0f;
        
        // Create the server.
        client = new TcpClient(this);
    	
    	
    }
    
    public void start() throws Exception
    {                    
            // Start the sensors.
            imu.resume();
            
            // Start the main controller thread.
            controllerThread.start();
            
    }
    
    public void stop()
    {
            // Stop the main controller thread.
            controllerThread.requestStop();
           
            // Stop the sensors.
            imu.pause();
            
            // Stop the server.
            client.stop();           
    }
    
    public void startClient(String serverIP)
    {
            client.start(serverIP);
    }
    
    public void stopClient()
    {
            client.stop();
    }
    
    public class ControllerThread extends Thread
	{
		@Override
		public void run()
		{		
			//boosting controller thread priority
			android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_URGENT_DISPLAY);
			controllerEnabled = true;
			
			lastTime=SystemClock.elapsedRealtimeNanos();
			
			while(controllerEnabled)
			{
				//check if imu thread has computed new measuraments
				if(!imu.getNewMeasurementsReady())
				{
					SystemClock.sleep(1);
					continue;
				}
				
				thrust=500;
				attitude=imu.getAttitude();
				relativeAttitude=imu.getRelativeAttitude();
				
				//dt computation
				currentTime=SystemClock.elapsedRealtimeNanos();
				dt=(float) ((currentTime-lastTime)/1000000000.0); //s
				motors.dt=(float) ((currentTime-lastTime)/1000000.0); //ms			

				output.pitch=pitchPID.getOutput(setPoint.pitch, relativeAttitude.theta,attitude.gy, dt);
				output.roll=rollPID.getOutput(setPoint.roll, relativeAttitude.fi,attitude.gx, dt);
				output.yaw=yawPID.getOutput(setPoint.yaw, relativeAttitude.psi,attitude.gz, dt);
				output.altitude=altitudePID.getOutput(setPoint.altitude, attitude.altitude, -attitude.vz, dt);
			
				if(Float.isInfinite(output.pitch) || Float.isInfinite(output.roll))
				{
					SystemClock.sleep(1);
					continue;
				}
				
				lastTime=currentTime;
//				output.pitch=PIDSaturation((float)(output.pitch*3.92f*R2DG),MAX_MOTOR_POWER);
//				output.roll=PIDSaturation((float)(output.roll*3.92f*R2DG),MAX_MOTOR_POWER);
//				output.yaw=PIDSaturation((float)(output.yaw*3.92f*R2DG),MAX_MOTOR_POWER);
//				output.altitude=PIDSaturation((float)(output.altitude*3.92f*R2DG),MAX_MOTOR_POWER);
//				N=PIDSaturation((output.pitch-output.yaw-output.altitude),MAX_MOTOR_POWER);
//				S=PIDSaturation((-output.pitch-output.yaw-output.altitude),MAX_MOTOR_POWER);
//				W=PIDSaturation((+output.roll+output.yaw-output.altitude),MAX_MOTOR_POWER);
//				E=PIDSaturation((-output.roll+output.yaw-output.altitude),MAX_MOTOR_POWER);
				
				//saturate output values
				output.pitch=PIDSaturation((float)(output.pitch*3.92f*R2DG),(int)thrust);
				output.roll=PIDSaturation((float)(output.roll*3.92f*R2DG),(int)thrust);
				output.yaw=PIDSaturation((float)(output.yaw*3.92f*R2DG),(int)thrust);
				output.altitude=PIDSaturation((float)(output.altitude*3.92f*R2DG),(int)thrust);
				//saturate linear combination of output values for + configuration
				N=PIDSaturation((output.pitch-output.yaw-output.altitude),(int)thrust);
				S=PIDSaturation((-output.pitch-output.yaw-output.altitude),(int)thrust);
				W=PIDSaturation((+output.roll+output.yaw-output.altitude),(int)thrust);
				E=PIDSaturation((-output.roll+output.yaw-output.altitude),(int)thrust);
				motors.thrust=thrust;
				timeWithoutPcRx= timeWithoutPcRx+dt;
				motors.pcRx=timeWithoutPcRx;
				if(regulatorEnabled)
				{
					motors.pwmN=(int) (1000+thrust+N);
					motors.pwmS=(int) (1000+thrust+S);					
					motors.pwmW=(int) (1000+thrust+W);
					motors.pwmE=(int) (1000+thrust+E);
					
				}else
				{
					thrust=0.0f;					
					motors.pwmW=(int) (1000);
					motors.pwmE=(int) (1000);
					motors.pwmN=(int) (1000);
					motors.pwmS=(int) (1000);
				}
				
				newMotorValue=true;
				
				//preparing log values for ground station						
				if(log != null)
                {
                        LogPoint p = new LogPoint();
                        p.time = currentTime/1000000 % INT_MAX;
                        p.yaw = attitude.psi;
                        p.pitch = attitude.theta;
                        p.roll = attitude.fi;
                        p.yawTarget = setPoint.yaw;
                        p.pitchTarget = setPoint.pitch;
                        p.rollTarget = setPoint.roll;
                        p.yawForce = output.yaw;
                        p.pitchForce = output.pitch;
                        p.rollForce = output.roll;
                        p.meanThrust = meanThrust;
                        p.nPower = motors.pwmN;
                        p.ePower = motors.pwmE;
                        p.sPower = motors.pwmS;
                        p.wPower = motors.pwmW;
                        p.altitude = attitude.altitude;
                        log.add(p);
                }
                
                // Transmit the current state to the computer.
                // If another component is transmitting (e.g. the video from the
                // camera), then do not transmit, because this would involve
                // waiting. This control loop should never be blocked!
                //if(!camera.isStreaming())
                {
                        stateSendDividerCounter++;
                        
                        if(stateSendDividerCounter % STATE_SEND_DIVIDER == 0)
                        {
                                String currStateStr = "" + (currentTime/1000000 % INT_MAX) +
                                                                          " " + attitude.psi*R2DG + " " + setPoint.yaw*R2DG + " " + output.yaw +
                                                                          " " + attitude.theta*R2DG + " " + setPoint.pitch*R2DG + " " + output.pitch +
                                                                          " " + attitude.fi*R2DG + " " + setPoint.roll*R2DG + " " + output.roll +
                                                                          " " + batteryVoltage +
                                                                          " " + 0 +
                                                                          " " + (regulatorEnabled?1:0) +
                                                                          " " + attitude.altitude + " " + setPoint.altitude + " " + output.altitude;
                                
                                        client.sendMessageNowOrSkip(currStateStr.getBytes(),
                                                                           TcpClient.TYPE_CURRENT_STATE);
                        }
                }
				
				
				SystemClock.sleep(8);
				
			}
		}
		
		
		public void requestStop()
         {
			 controllerEnabled = false;
         }
		 
		 private boolean controllerEnabled;

	}
    
    
    private float PIDSaturation(float val, int max)
    {
    	if(val>max)
    	{
    		return max;
    	}else if(val<-max)
    	{
    		return -max;
    	}
    	
    	else return val;
    }
    
    public void onConnectionEstablished()
    {
            // Reset the orientation.
            imu.setCurrentStateAsZero();
    }

    public void onConnectionLost()
    {
            // Emergency stop of the quadcopter.
            emergencyStop();
    }
    
    public void onMessageReceived(String message)
    {               
            // Reset the timer.
            timeWithoutPcRx = 0.0f;
            motors.message=message;
            // Interpret the message, and do an action corresponding to the enclosed
            // order.
            if(message.equals("heartbeat"))
            {
                    // Do nothing, this is just to reset the timer.
            }
            else if(message.equals("emergency_stop"))
                    emergencyStop();
            else if(message.startsWith("command "))
            {
                    String[] values = message.replace("command ", "").split("\\s");
                    
                    if(values.length == 4)
                    {
                    	setPoint.altitude = -(float)Integer.parseInt(values[0])/10000.0f;
                    	
                    	setPoint.yaw = (float) (Float.parseFloat(values[1])/R2DG);
                    	setPoint.pitch = (float) (Float.parseFloat(values[2])/R2DG);
                    	setPoint.roll = (float) (Float.parseFloat(values[3])/R2DG);
                            
                            //Log.d("dp", "meanThrust: " + meanThrust);
                            
                            // Reset the integrators if the motors are off.
                            if(Integer.parseInt(values[0]) == 0)
                            {
                            	yawPID.resetIntegrator();
                            	altitudePID.resetIntegrator();
                            	pitchPID.resetIntegrator();
                            	rollPID.resetIntegrator();
                            }
                    }
            }
            else if(message.startsWith("regulator_coefs "))
            {
            	//uncomment in case of online controll coefficent update
//                    String[] values = message.replace("regulator_coefs ", "").split("\\s");
//                    
//                    if(values.length == 12)
//                    {
//                            float yawP = Float.parseFloat(values[0]);
//                            float yawI = Float.parseFloat(values[1]);
//                            float yawD = Float.parseFloat(values[2]);
//                            float pitchP = Float.parseFloat(values[3]);
//                            float pitchI = Float.parseFloat(values[4]);
//                            float pitchD = Float.parseFloat(values[5]);
//                            float rollP = Float.parseFloat(values[6]);
//                            float rollI = Float.parseFloat(values[7]);
//                            float rollD = Float.parseFloat(values[8]);
//                            float altiP = Float.parseFloat(values[9]);
//                            float altiI = Float.parseFloat(values[10]);
//                            float altiD = Float.parseFloat(values[11]);
//                            yawRegulator.setCoefficients(yawP, yawI, yawD);
//                            pitchRegulator.setCoefficients(pitchP, pitchI, pitchD);
//                            rollRegulator.setCoefficients(rollP, rollI, rollD);
//                            altitudeRegulator.setCoefficients(altiP, altiI, altiD);
//                    }
            	
            	//break;
            }
            else if(message.equals("regulator_state on"))
                    regulatorEnabled = true;
            else if(message.equals("regulator_state off"))
            {
                    regulatorEnabled = false;
            }
            else if(message.equals("log on"))
            {
                    // Create a log file.
                    log = new ArrayList<LogPoint>();
                    
                    client.sendMessage("Logging started.".getBytes(), TcpClient.TYPE_TEXT);
            }
            else if(message.equals("log off"))
            {
                    // Generate the string.
                    StringBuilder sb = new StringBuilder();
                    
                    for(int i=0; i<log.size(); i++)
                    {
                            LogPoint p = log.get(i);
                            sb.append("" + p.time + " "
                                              + p.yaw + " " + p.pitch + " " + p.roll + " "
                                              + p.yawTarget  + " " + p.pitchTarget + " "
                                              + p.rollTarget + " " + p.yawForce + " "
                                              + p.pitchForce + " " + p.rollForce + " "
                                              + p.meanThrust + " " + p.nPower + " "
                                              + p.ePower + " " + p.sPower + " " + p.wPower + " "
                                              + p.altitude + "\n");
                    }

                    // Send the log.
                    client.sendMessage(sb.toString().getBytes(), TcpClient.TYPE_LOG);
                    client.sendMessage("Logging finished.".getBytes(), TcpClient.TYPE_TEXT);
                    
                    // Stop logging.
                    log = null;
            }
            else if(message.equals("orientation_reset"))
            {
            	Log.w("dp", "orientation reset!");    
            	imu.setCurrentStateAsZero();
            }
            else if(message.startsWith("fpv "))
            {
            	//uncomment in case of fpv implementation
//                    String newStateString = message.replace("fpv ", "");
//                    
//                    if(newStateString.equals("stop"))
//                            camera.stopStreaming();
//                    else if(newStateString.equals("sd"))
//                    {
//                            camera.setFrameSize(activity, false);
//                            camera.startStreaming(client);
//                    }
//                    else if(newStateString.equals("hd"))
//                    {
//                            camera.setFrameSize(activity, true);
//                            camera.startStreaming(client);
//                    }
            	
            	//break;
            }
//            else if(message.equals("take_picture"))
//                    camera.takePicture(client);
            else if(message.startsWith("altitude_lock "))
            {
                    String newStateString = message.replace("altitude_lock ", "");
                    
                    if(newStateString.equals("on"))
                    {
//                            altitudeTarget = ; //get actual read from sonar
//                            //altitudeRegulator.setAPriori((float)motorsPowers.getMean());
//                            altitudeLockEnabled = true;
                    }
                    else if(newStateString.equals("off"))
                            altitudeLockEnabled = false;
            }
    }
    
    private void emergencyStop()
    {
            Log.w("dp", "Emergency stop!");
            regulatorEnabled = false;
    }
    
    public void onBatteryVoltageArrived(float batteryVoltage)
    {
            this.batteryVoltage = batteryVoltage;
            
            // Reset the timer.
    }
    
    private class LogPoint
    {
            public long time;
            public float yaw, pitch, roll, yawTarget, pitchTarget, rollTarget,
                                     yawForce, pitchForce, rollForce, meanThrust,
                                     nPower, ePower, sPower, wPower, altitude;
    }
    
    public class SetPoint
    {
    	public float yaw, pitch, roll, altitude;
    }
    
    public class Output
    {
    	public float yaw, pitch, roll, altitude;
    }
    
    public class Motors
	{
		public int pwmN,pwmE,pwmW,pwmS;
		public float dt,pcRx, outputAltitude, thrust;
		public String message;
	}
    
    public Motors getMotors()
    {
    	newMotorValue=false;
    	
    	return motors;
    }
    
    public boolean getnewMotorValue()
	{
		return newMotorValue;
	}
    
    public SetPoint getSetPoint()
    {
    	return setPoint;
    }    
    
	public Attitude getAttitude()
	{
		return attitude;
	}
	public Attitude getRelativeAttitude()
	{
		return relativeAttitude;
	}
	
	public Output getOutput()
	{
		return output;
	}
	
	public float getDerivative()
	{
		return rollPID.getDerivator();
	}
	public void updatePID(float _kp, float _ki, float _kd)
	{
		pitchPID.setCoefficients(_kp, _ki, _kd);
		rollPID.setCoefficients(_kp, _ki, _kd);
	}
	
	public void setSonar(float sonarRaw)
	{
		sonar=sonarRaw/0.0032f;
		imu.setSonarRaw(sonar);
	}
	public float getSonar()
	{
		return sonar;
	}
	public void resetGps()
	{
		imu.resetGps();
	}
	
	private Attitude attitude, relativeAttitude;
	  
    private imuReading imu;
    
    private PIDRegulator pitchPID;
    private PIDRegulator rollPID;
    private PIDRegulator yawPID;
    private PIDRegulator altitudePID;
    
    private boolean newMotorValue;
    
    private float kp,ki,kd,LP;
    
    private float dt;
    private long lastTime, currentTime;
    
    
    private Motors motors;
    private SetPoint setPoint;
    private Output output;
    private float sonar;
    private ControllerThread controllerThread;
	
	private TcpClient client;
    
    private float meanThrust, N, S, E, W, batteryVoltage, timeWithoutPcRx;
    private float thrust=0.0f;
    
    
    private boolean regulatorEnabled, altitudeLockEnabled;
    private ArrayList<LogPoint> log;
    private int stateSendDividerCounter;

}
