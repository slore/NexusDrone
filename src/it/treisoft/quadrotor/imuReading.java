/**
 * 
 */
package it.treisoft.quadrotor;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.SystemClock;


/**
 * @author Lorenzo Aldrovandi lorenzo.accesso@gmail.com
 *
 */
public class imuReading implements SensorEventListener{
    
	//costante di conversione da Radianti a Gradi
	private static final double R2DG=360.0f/2.0f/Math.PI;	
	private static final double EARTH_RADIUS = 6371000;	
	private static final float gAcceleration=9.81f;
	
	//magnetometer scale constants obtain through calibration process
	private static final float scaleX = 0.9156f;
	private static final float scaleY = 1.1163f;
	private static final float scaleZ = 0.9881f;
	
    // orientation angles from accel and magnet
    private float fi, theta, psi, fiZero, psiZero, thetaZero,altitudeZero;
	
    // magnetic field vector
    private float[] magnet = new float[3];
 
    // accelerometer vector
    private float[] accel = new float[3];
    //aNorm= normalized and calibrated accelerometer vector
    private float[] aNorm = new float[3];
    //linear acceleration vector
    private float[] linAccel=new float[3];
    
    // angular speeds from gyro
    private float[] gyro = new float[3];
    private float gyroZ;
    
    //Mahony filter variable
    private float fiIntegral, thetaIntegral, psiIntegral;
    //euler angles compute from accelerometer and magnetometer only
    private float fiAccel, thetaAccel, psiAccel;
    //Mahony's filter coefficients
    private float _KI=0.06f;
    private float _KP=40.0f;
   
    private float dt,dt_gps, lastGpsElevation,last_zSpeed, absoluteElevation, elevationZero;
    
    private double timeStamp, current_gps;
    private double lastTimeStamp, last_gps;
    private double timeLapse;

    //daniele grifi's position filter variables
    private float[] aTrueDot= new float[3];
    private float[] aTrue= new float[3];
    private float[] vTrueDot= new float[3];
    private float[] vTrue= new float[3];
    private float[] posTrueDot= new float[3];
    private float[] posTrue= new float[3];
    private float[] pGps= new float[3];
    private float[] deltaTrue= new float[3];
    private float[] deltaTrueDot= new float[3];
    private float[] posMis= new float[3];
    private float[] deltaZero= new float[3];
    private float[] pTrueZero= new float[3];    
    //daniele grifi's filter coefficents
    private static final float ka=0.03f;
    private static final float kv=1f;
    private static final float kp=0.01f;
    //z axis complementary filter coefficents
    private static final float k1=14f;
    private static final float k2=14f;
    private static final float k3=0.2f;
   
    private float[] rotationMatrix= new float[9];
    private float z1,dz1,z2,dz2,b,db,h_est, vz, last_vz,lastSonar, pos_z_last;
    private boolean newAccelMesuramentReady, newGpsReady;
    private boolean newGyroMesuramentReady;
    private boolean newMeasurementsReady;
    private boolean first;
    
    
    private ImuThread imuThread;
    
    private Attitude attitude, relativeAttitude;    
    private SensorManager mSensorManager = null;
    private LocationManager locationManager;
    private LocationListener locationListener;	

    public imuReading(Context context)
	{
    	
    	//initialize rotation matrix to identity
    	rotationMatrix[0]=1f;rotationMatrix[1]=0f;rotationMatrix[2]=0f;
    	rotationMatrix[3]=0f;rotationMatrix[4]=1f;rotationMatrix[5]=0f;
    	rotationMatrix[6]=0f;rotationMatrix[7]=0f;rotationMatrix[8]=1f;
    	
    	newGpsReady=true;
    	
    	// initialize new measurements flags to false
    	newAccelMesuramentReady=false;
    	newGyroMesuramentReady=false;
    	newMeasurementsReady=false;
    	first=true;
    	
    	//initialize attitude structs
    	attitude=new Attitude();
    	relativeAttitude=new Attitude();
    	
    	// initialize last angles variables to zero        
    	attitude.fi=0.0f;
    	attitude.theta=0.0f;
    	attitude.psi=0.0f;
    	
    	 // initialize integral variables to zero
        fiIntegral=0;
        thetaIntegral=0;
        psiIntegral=0;
    	
    	//sonar begins to stream data from 20cm to 670cm
    	attitude.sonarRaw=-0.2f;
    	lastSonar=-0.2f;
    	
    	fiZero=0.0f;
    	thetaZero=0.0f;
    	psiZero=0.0f;
    	altitudeZero=0.0f;
    	
    	attitude.latitudeZero=0f;
    	attitude.longitudeZero=0f;
        
       
        
        lastTimeStamp=0;
        
        mSensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);

        
        locationManager = (LocationManager) context.getSystemService(Context.LOCATION_SERVICE);
       
        locationListener = new LocationListener()
                {
                public void onLocationChanged(Location location)
                        {
			                	current_gps=SystemClock.elapsedRealtimeNanos();
			    				dt_gps=(float) ((current_gps-last_gps)/1000000000.0);;
			    				attitude.gps_dt=dt_gps*1000;
			    				last_gps=current_gps;               				
                				
                				attitude.gpsElevation = (float) location.getAltitude();
                                attitude.gpsAccuracy = location.getAccuracy();
                                attitude.longitude = location.getLongitude();
                                attitude.latitude = location.getLatitude();
                                
                                attitude.nSatellites = (Integer) location.getExtras().get("satellites");
                                attitude.bearing=location.getBearing();
                                attitude.speed=location.getSpeed(); 
                                // Convert longitude+latitude to x+y (using the small angles
                                // approximation: sin(x)~=x).
                                attitude.xPos = (float) (EARTH_RADIUS * (attitude.longitude-attitude.longitudeZero)/R2DG);
                                attitude.yPos = (float) (EARTH_RADIUS * (attitude.latitude-attitude.latitudeZero) /R2DG);
                                
                                // Convert heading+speed to speedx+speedy.
                                attitude.xSpeed = location.getSpeed() *(float)(Math.cos(location.getBearing()/R2DG));
                                attitude.ySpeed = location.getSpeed() * (float)(Math.sin(location.getBearing()/R2DG));
                                newGpsReady=true;
                        }
                        
                        public void onStatusChanged(String provider, int status, Bundle extras)
                        {
                                attitude.nSatellites = (Integer) extras.get("satellites");
                                attitude.gpsStatus = status;
                        }
                        
                        public void onProviderEnabled(String provider) {}
                        public void onProviderDisabled(String provider){}
						
                };
                locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER,
                        0, 0, locationListener);
                
                
	}
    
    public void resume()
    {
    	
    	// get sensorManager and initialize sensor listeners    
        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_FASTEST);
         
        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                SensorManager.SENSOR_DELAY_FASTEST);
         
        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_FASTEST);
        
        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION),
                SensorManager.SENSOR_DELAY_FASTEST);
        
        
        
        imuThread=new ImuThread();
    	
    	imuThread.start();
    	
            
        
    }
    
    public void pause()
    {
                
    	imuThread.requestStop();
    	// Disable the inertial sensors.
    	//mSensorManager.unregisterListener(this);
    }
    
	@Override
	public void onSensorChanged(SensorEvent event) {
		
		switch(event.sensor.getType()) {
	    case Sensor.TYPE_ACCELEROMETER:
	    	accel[0]=-event.values[1];
	    	accel[1]=-event.values[0];
	    	accel[2]=event.values[2];
	    	
	    	//normalized and calibrated accelerometer reading
	    	aNorm[0]=0.1025f*accel[0]-0.0007f*accel[1]-0.0059f*accel[2]+0.0176f;
	    	aNorm[1]=0.0023f*accel[0]+0.1025f*accel[1]-0.0088f;
	    	aNorm[2]=0.0002f*accel[0]+0.0005f*accel[1]+0.1022f*accel[2]+0.0497f;
	       
	    	//update new mesurament flag 
	        newAccelMesuramentReady=true;
	        
	        break;
	 
	    case Sensor.TYPE_GYROSCOPE:
	        
	    	gyro[0]=event.values[1];
	    	gyro[1]=event.values[0];
	    	gyro[2]=-event.values[2];
	    	
	    	//controller needs angular velocity for Derivative part
	    	attitude.gx=gyro[0];
	    	attitude.gy=gyro[1];
	    	attitude.gz=gyro[2];
	    	
	    	//update new mesurament flag 
	        newGyroMesuramentReady=true;
	    	
	    	break;
	 
	    case Sensor.TYPE_MAGNETIC_FIELD:
	               
	    	//normalized and calibrated magnet reading
	    	magnet[0]=-event.values[1]*scaleX;
	    	magnet[1]=-event.values[0]*scaleY;
	    	magnet[2]=event.values[2]*scaleZ;    	
	    			
	        break;
	    case Sensor.TYPE_LINEAR_ACCELERATION:
	    	
	    	linAccel[0]=-event.values[1];
	    	linAccel[1]=-event.values[0];
	    	linAccel[2]=event.values[2];	    	
	    	break;
	        
	    }
	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		
		
	}
	
	private void computeRotationMatrix()
	{
		rotationMatrix[0]=(float) Math.cos(attitude.psi)*(float) Math.cos(attitude.theta);
		rotationMatrix[1]=-(float) Math.sin(attitude.psi)*(float) Math.cos(attitude.theta)+(float) Math.sin(attitude.theta)*(float) Math.cos(attitude.psi)*(float) Math.sin(attitude.fi);
		rotationMatrix[2]=(float) Math.sin(attitude.fi)*(float) Math.sin(attitude.psi)+(float) Math.sin(attitude.theta)*(float) Math.cos(attitude.psi)*(float) Math.cos(attitude.fi);
    	rotationMatrix[3]=(float) Math.sin(attitude.psi)*(float) Math.cos(attitude.theta);
    	rotationMatrix[4]=(float) Math.cos(attitude.fi)*(float) Math.cos(attitude.psi)+(float) Math.sin(attitude.psi)*(float) Math.sin(attitude.fi)*(float) Math.sin(attitude.theta);
    	rotationMatrix[5]=-(float) Math.cos(attitude.psi)*(float) Math.sin(attitude.fi)+(float) Math.sin(attitude.psi)*(float) Math.cos(attitude.fi)*(float) Math.sin(attitude.theta);
    	rotationMatrix[6]=-(float) Math.sin(attitude.theta);
    	rotationMatrix[7]=(float) Math.cos(attitude.theta)*(float) Math.sin(attitude.fi);
    	rotationMatrix[8]=(float) Math.cos(attitude.theta)*(float) Math.cos(attitude.fi);
    }
	
	private void calculateAccMagEuAngles() {
			// tan(theta)=-Ax/Az
			thetaAccel=(float)Math.atan2(-aNorm[0], aNorm[2]);			
			//tan(fi)=Ax/sqrt(Ay^2+Az^2)
			fiAccel=(float)Math.atan2(aNorm[1], (float)Math.sqrt((aNorm[0]*aNorm[0]+aNorm[2]*aNorm[2])));					
			psiAccel=(float)Math.atan2((-magnet[1]*Math.cos(attitude.fi)+magnet[2]*Math.sin(attitude.fi)), magnet[0]*Math.cos(attitude.theta)+magnet[1]*Math.sin(attitude.theta)*Math.sin(attitude.fi)+magnet[2]*Math.sin(attitude.theta)*Math.cos(attitude.fi));
	}
	
	public class ImuThread extends Thread
	{
		@Override
		public void run()
		{
			android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_URGENT_DISPLAY);
			_continue=true;
			
			while(_continue)
			{
				//estimation should wait that all sensors expose new data 
				//but practically is better to have a very low dt to avoid jitter problem (android is not a real-time OS)
				if(false)     //if(!newAccelMesuramentReady && !newGyroMesuramentReady)
				{
					SystemClock.sleep(1);
					continue;
				}
				
				newAccelMesuramentReady=false;
				newGyroMesuramentReady=false;
				
				//skip first loop to update lastTS
				if(first)
				{
					lastTimeStamp=SystemClock.elapsedRealtimeNanos();
					first=false;
					continue;
				}
				
				//computing time since last iteration
				timeStamp=SystemClock.elapsedRealtimeNanos();
				dt=(float) ((timeStamp-lastTimeStamp)/1000000000.0);
				lastTimeStamp=timeStamp;	           
				
				calculateAccMagEuAngles();
		    	
		    	//mahony filter THETA
		    	thetaIntegral= thetaIntegral + _KI*dt*(thetaAccel-attitude.theta);
		    	attitude.theta=_wrap_pi((1-_KP*dt)*attitude.theta+_KP*dt*thetaAccel+(gyro[0]+thetaIntegral)*dt);
		    	
		    	//mahony filter FI
		    	fiIntegral= fiIntegral + _KI*dt*(fiAccel-attitude.fi);
		    	attitude.fi=(1-_KP*dt)*attitude.fi+_KP*dt*fiAccel+(gyro[1]+fiIntegral)*dt;
		    	
		    	computeRotationMatrix();
		    	//raw gyro z axis integration
		    	gyroZ=rotationMatrix[6]*gyro[0]+rotationMatrix[7]*gyro[1]+rotationMatrix[8]*gyro[2];
		    			    	
		    	//mahony filter psi
		    	psiIntegral= psiIntegral + _KI*dt*(psiAccel-attitude.psi);
		    	attitude.psi=(1-_KP*dt)*attitude.psi+_KP*dt*psiAccel+(gyroZ+psiIntegral)*dt;
		    	
		    	// Make the measurements relative to the user-defined zero orientation.
		    	attitude.gz=gyroZ;
		    	
		    	relativeAttitude.psi = attitude.psi-psiZero;
                relativeAttitude.theta = attitude.theta-thetaZero;
                relativeAttitude.fi = attitude.fi-fiZero;	    	

		    	attitude.az=(rotationMatrix[6]*(accel[0]-0.36f)+rotationMatrix[7]*accel[1]+rotationMatrix[8]*(accel[2]+0.44f)-gAcceleration);
				
				float sonar_diff=Math.abs(attitude.sonarRaw-lastSonar);

				if (sonar_diff>0.2f)
				{
					attitude.altitude = lastSonar;
				}
				else
				{
					attitude.altitude = attitude.sonarRaw;
					lastSonar= attitude.sonarRaw;
				}
				
				/* simple lowpass sonar filtering */
				attitude.altitude = 0.8f * attitude.altitude + 0.2f * pos_z_last;
				pos_z_last = attitude.altitude;
				//Vz estimation from sonar change position
				vz=(attitude.altitude-lastSonar)/dt;
				//LP filter to smooth speed
				attitude.vz=LowPassFilter(vz, last_vz, 0.4f, dt);
				//updating sonar LP variables
				lastSonar=attitude.altitude;
				last_vz=attitude.vz;
				
				//z axis complementary filter
				dz1 = z2 - k1*(z1-attitude.altitude);
				db=-k3*(z2-attitude.vz);
				b=b+db*dt;
				dz2 = attitude.az - k2*(z2-attitude.vz)+b;
				z1 = z1 + dz1*dt;
				z2 = z2 + dz2*dt;
				attitude.altitude = z1;
				attitude.vz = z2;
				//End compl. Filter
				
				relativeAttitude.elevation=attitude.altitude+altitudeZero;
				
				//daniele grifi's position filter
				aTrueDot[0]=-ka*(rotationMatrix[0]*(attitude.xSpeed-vTrue[0])+rotationMatrix[3]*(attitude.ySpeed-vTrue[1])+rotationMatrix[6]*(attitude.vz-vTrue[2]));
				aTrueDot[1]=-ka*(rotationMatrix[1]*(attitude.xSpeed-vTrue[0])+rotationMatrix[4]*(attitude.ySpeed-vTrue[1])+rotationMatrix[7]*(attitude.vz-vTrue[2]));
				aTrueDot[2]=-ka*(rotationMatrix[2]*(attitude.xSpeed-vTrue[0])+rotationMatrix[5]*(attitude.ySpeed-vTrue[1])+rotationMatrix[8]*(attitude.vz-vTrue[2]));
				
				aTrue[0]=aTrue[0]+aTrueDot[0]*dt;
				aTrue[1]=aTrue[1]+aTrueDot[1]*dt;
				aTrue[2]=aTrue[2]+aTrueDot[2]*dt;
				
				vTrueDot[0]=rotationMatrix[0]*(linAccel[0]-aTrue[0])+rotationMatrix[1]*(linAccel[1]-aTrue[1])+rotationMatrix[2]*(linAccel[2]-aTrue[2])+kv*(attitude.xSpeed-vTrue[0]);
				vTrueDot[1]=rotationMatrix[3]*(linAccel[0]-vTrue[0])+rotationMatrix[4]*(linAccel[1]-aTrue[1])+rotationMatrix[5]*(linAccel[2]-aTrue[2])+kv*(attitude.ySpeed-vTrue[1]);
				vTrueDot[2]=rotationMatrix[6]*(linAccel[0]-aTrue[0])+rotationMatrix[7]*(linAccel[1]-aTrue[1])+rotationMatrix[8]*(linAccel[2]-aTrue[2])+kv*(attitude.vz-vTrue[2]);
				
				vTrue[0]=vTrue[0]+vTrueDot[0]*dt;
				vTrue[1]=vTrue[1]+vTrueDot[1]*dt;
				vTrue[2]=vTrue[2]+vTrueDot[2]*dt;
				
				deltaTrue[0]=deltaTrue[0]+vTrue[0]*dt;
				deltaTrue[1]=deltaTrue[1]+vTrue[1]*dt;
				deltaTrue[2]=deltaTrue[2]+vTrue[2]*dt;
				
				if(newGpsReady)
				{
					newGpsReady=false;
					pGps[0]=attitude.xPos;
					pGps[1]=attitude.yPos;
					
					deltaZero[0]=deltaTrue[0];
					deltaZero[1]=deltaTrue[1];
					
					pTrueZero[0]=posTrue[0];
					pTrueZero[1]=posTrue[1];
					
					posMis[0]=(pTrueZero[0]+pGps[0])/2.0f+deltaTrue[0]-deltaZero[0];
					posMis[1]=(pTrueZero[1]+pGps[1])/2.0f+deltaTrue[1]-deltaZero[1];
					
				}else
				{
					posMis[0]=(pTrueZero[0]+pGps[0])/2.0f+deltaTrue[0]-deltaZero[0];
					posMis[1]=(pTrueZero[1]+pGps[1])/2.0f+deltaTrue[1]-deltaZero[1];
				}
				
				posTrueDot[0]=vTrue[0]+kp*(posMis[0]-posTrue[0]);
				posTrueDot[1]=vTrue[1]+kp*(posMis[1]-posTrue[1]);				
				
				posTrue[0]=posTrue[0]+posTrueDot[0]*dt;
				posTrue[1]=posTrue[1]+posTrueDot[1]*dt;
				//end position filter
				
				attitude.x=posTrue[0];
				attitude.y=posTrue[1];
				attitude.vx=vTrue[0];
				attitude.vy=vTrue[1];
				
				
				
				newMeasurementsReady=true;
			}
		}
		
		
			
		private void requestStop()
		{
			_continue=false;
		}
		
		private boolean _continue;
	}
	
	public void setCurrentStateAsZero()
	{
		
		psiZero=attitude.psi;
		thetaZero=attitude.theta;
		fiZero=attitude.fi;
		altitudeZero=attitude.altitude;
		
	}
	
	float LowPassFilter(float value, float last, float cutOff, float dt_filt){
		  float Tf= (float) (1.0/(cutOff*2f*3.14f)); //Cutoff frequency Hz
		  float a = dt_filt/(dt_filt+Tf);
		  float val = a*value+(1-a)*last;
		  return val;
		}
	
	float _wrap_pi(float bearing)
	{
		/* value is inf or NaN */
		if (Float.isInfinite(bearing)) {
			return bearing;
		}

		int c = 0;
		while (bearing >= Math.PI) {
			bearing -= 2*Math.PI;
			if (c++ > 3)
				return Float.NaN;
		}

		c = 0;
		while (bearing < -Math.PI) {
			bearing += 2*Math.PI;
			if (c++ > 3)
				return Float.NaN;
		}

		return bearing;
	}

	
	public class Attitude
	{
		//very dirty structure but i currently use it to show data in mainActivity
		
		public float fi, theta, psi, gx,gy,gz,x,y,vx,vy, baroElevation;
		public float sonarRaw,vz,az,altitude,elevation, zSpeed, gps_dt;
		public long time;
		
		 public float gpsElevation, gpsAccuracy, nSatellites, bearing, speed,
		 				xPos, yPos, xSpeed, ySpeed, gpsStatus;
		 public double longitude,longitudeZero, latitude, latitudeZero;
	}
	
	public Attitude getAttitude()
	{
		//newMeasurementsReady=false;
		
		return attitude;
	}
	
	public Attitude getRelativeAttitude()
	{
		newMeasurementsReady=false;
		
		return relativeAttitude;
	}
	
	public void setSonarRaw(float sonarRaw)
	{
		attitude.sonarRaw=-sonarRaw/100.0f; //m
	}
	public boolean getNewMeasurementsReady()
	{
		return newMeasurementsReady;
	}
	
	public void resetGps()
	{
		attitude.longitudeZero=attitude.longitude;
		attitude.latitudeZero=attitude.latitude;
	}

}
