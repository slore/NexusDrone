package it.treisoft.quadrotor;

import it.treisoft.quadrotor.Controller.Motors;
import it.treisoft.quadrotor.Controller.Output;
import it.treisoft.quadrotor.Controller.SetPoint;
import it.treisoft.quadrotor.imuReading.Attitude;
import it.treisoft.quadrotor.Controller;


import ioio.lib.api.AnalogInput;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.android.IOIOActivity;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;
import android.R.string;
import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.SystemClock;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;


public class MainActivity extends IOIOActivity {
	
	//Radians to degree conversion costant
	public static final double R2DG=360.0f/2.0f/Math.PI;
	
	//display variables	
	private TextView TVfi, TVtheta, TVpsi, tvN, tvE, tvW, tvS;
	private Button bStart, bStop, bArm, bDisarm, bStartLog, bStopLog, bResetGps;	
	//update display variables
	public Handler mHandler;
	public static final int TIME_CONSTANT = 10;
	private Timer updateTimer = new Timer();    
	private Attitude attitude, relativeAttitude;
	private Motors motors;
	private SetPoint setPoint;
	private Output output;
	private String strControl, strPsi, strFi, strTheta;
	
	private boolean armed, controlStarted, logging;
	private int baseValuePWM;
	
		
    public Controller controller;
    public logger log;
    
    public void onCreate(Bundle savedInstanceState) {
        
    	super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        
        //initialize textviews variables
        TVfi=(TextView)findViewById(R.id.TVfi);
        TVtheta=(TextView)findViewById(R.id.TVtheta);
        TVpsi=(TextView)findViewById(R.id.TVpsi);
        tvN=(TextView)findViewById(R.id.tvN);
        tvS=(TextView)findViewById(R.id.tvS);
        tvE=(TextView)findViewById(R.id.tvE);
        tvW=(TextView)findViewById(R.id.tvW);
        
        // initialize buttons variables
        bStart=(Button)findViewById(R.id.bStart);
        bResetGps=(Button)findViewById(R.id.bResetGps);
        bStop=(Button)findViewById(R.id.bStop);
        bStartLog=(Button)findViewById(R.id.bStartLog);
        bStopLog=(Button)findViewById(R.id.bStopLog);
        bArm=(Button)findViewById(R.id.bArm);
        bDisarm=(Button)findViewById(R.id.bDisarm);
        
        //initialize state machine variables
        armed=false;
        controlStarted=false;
        logging=false;       
        
        registOnClickListeners();         
        mHandler = new Handler(); 
       
        //initialiaze controller
        controller=new Controller(this);
    }
    
    private void registOnClickListeners() {
    	bResetGps.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                controller.resetGps();
                Toast toast = Toast.makeText(getApplicationContext(), "gps reset", Toast.LENGTH_SHORT);
                toast.show();
            }
        });
        
        bStartLog.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                if(!logging)
                {                	
                	log=new logger();
                	log.startLog();
                	Toast toast = Toast.makeText(getApplicationContext(), "log started", Toast.LENGTH_SHORT);
                	toast.show();
                	logging = true;
                }
            }
        });
             
        bStopLog.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                if(logging)
                {
                	logging = false;
                	log.stoptLog();
                	Toast toast = Toast.makeText(getApplicationContext(), "log stopped", Toast.LENGTH_SHORT);
        			toast.show();
                }
            	
            	
            }
        });
        
        bArm.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                if(!armed)
                {
	            	armed=true;
	            	baseValuePWM=1000;
	            	bArm.setClickable(false);
	            	bDisarm.setClickable(true);
	            	SystemClock.sleep(2000);
	            	Toast toast = Toast.makeText(getApplicationContext(), "Motors armed", Toast.LENGTH_SHORT);
    				toast.show();
                }
            }
        });
        
        bDisarm.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                if(armed)
                {
	            	armed=false;
	            	baseValuePWM=0;
	            	bArm.setClickable(true);
	            	bDisarm.setClickable(false);
	            	Toast toast = Toast.makeText(getApplicationContext(), "Motors DISarmed", Toast.LENGTH_SHORT);
    				toast.show();
                }
            }
        });
        
        bStart.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                if(!controlStarted)
                {
                	if(armed)
                	{
                		
                		strControl="Starting controller";                		
                		Toast toast = Toast.makeText(getApplicationContext(), strControl, Toast.LENGTH_SHORT);
        				toast.show();                		
                		
                        try
                        {
                        	//start controller
                        	controller.start();
                        	
                        	//start update display task
                        	updateTimer.scheduleAtFixedRate(new updateTask(),
                                    3000, TIME_CONSTANT);
                            SystemClock.sleep(2000);
                            
                            controller.startClient("192.168.43.167");
                            
                            controlStarted=true;
                            toast = Toast.makeText(getApplicationContext(), "controller started", Toast.LENGTH_SHORT);
            				toast.show();
            				bStart.setClickable(false);
        	            	bStop.setClickable(true);
                        }
                        catch (Exception e)
                        {
                        	toast = Toast.makeText(getApplicationContext(), "controller NOT started", Toast.LENGTH_SHORT);
            				toast.show();
                            e.printStackTrace();
                        }
                		
                	}else{
                		strControl="Arm motor first";
                		Toast toast = Toast.makeText(getApplicationContext(), strControl, Toast.LENGTH_SHORT);
        				toast.show();
                	}
                	
                	            	
	            	
                }
            }
        });
        
        bStop.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                if(controlStarted)
                {
                	controlStarted=false;
                	//stopping controller
                    controller.stop();
                    controller.stopClient();
                	Toast toast = Toast.makeText(getApplicationContext(), "controller stopped", Toast.LENGTH_SHORT);
    				toast.show();
    				bStart.setClickable(true);
    				bStop.setClickable(false);
                }
            }
        });
		
	}

	@Override
    protected void onResume()
    {
        super.onResume();        
        // Prevent sleep mode.
        getWindow().addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    }
    
    @Override
    protected void onPause()
    {
        super.onPause();
        // Reallow sleep mode.
        getWindow().clearFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);      
        updateTimer.cancel();
    }
    
    
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();
        if (id == R.id.action_settings) {
            return true;
        }
        return super.onOptionsItemSelected(item);
    }
    
    class Looper extends BaseIOIOLooper {
		
    	private AnalogInput sonar;
    	//motor pin Variables
    	private PwmOutput pwmOutputN;
		private PwmOutput pwmOutputE;
		private PwmOutput pwmOutputW;
		private PwmOutput pwmOutputS;

		@Override
		public void setup() throws ConnectionLostException, InterruptedException {
			sonar = ioio_.openAnalogInput(40);
			//motor N is attached to pin11 and so on..
			pwmOutputN = ioio_.openPwmOutput(11, 400);
			pwmOutputS = ioio_.openPwmOutput(12, 400);
			pwmOutputE = ioio_.openPwmOutput(13, 400);
			pwmOutputW = ioio_.openPwmOutput(14, 400);
		}

		@Override
		public void loop() throws ConnectionLostException, InterruptedException {
			
			if(!controlStarted)
			{
				pwmOutputN.setPulseWidth(baseValuePWM);
				pwmOutputS.setPulseWidth(baseValuePWM);
				pwmOutputE.setPulseWidth(baseValuePWM);
				pwmOutputW.setPulseWidth(baseValuePWM);
				Thread.sleep(1);
			}else
			{
				try {
//					controller.setSonar(sonar.getVoltage());
					controller.setSonar(0f);
					pwmOutputN.setPulseWidth(motors.pwmN);
					pwmOutputS.setPulseWidth(motors.pwmS);
					pwmOutputE.setPulseWidth(motors.pwmE);
					pwmOutputW.setPulseWidth(motors.pwmW);
					Thread.sleep(1);
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}

		@Override
		public void disconnected() { 
			
		}
	}

	@Override
	protected IOIOLooper createIOIOLooper() {
		
		return new Looper();
	}


	
	class updateTask extends TimerTask {
	    public void run() {	      	
	    	
	    		attitude=controller.getAttitude();
	    		relativeAttitude=controller.getRelativeAttitude();
	    		setPoint=controller.getSetPoint();
	    		output=controller.getOutput();
	    		motors=controller.getMotors();
	    		if(logging)
	    		{
	    			log.append(attitude, setPoint, motors);
	    		}
	    		
	    		mHandler.post(updateDisplayTask);
	    		
	    		    	
	    }
	}

	public void updateDisplay() {
		
		tvN.setText("N: "+motors.pwmN);
		tvS.setText("S: "+motors.pwmS);
		tvE.setText("E: "+motors.pwmE);
		tvW.setText("W: "+motors.pwmW);
		
		strFi=String.format("fi: %.3f°\nrel.fi: %.3f°\nsp.fi: %.3f°\n", attitude.fi*R2DG, relativeAttitude.fi*R2DG, setPoint.roll*R2DG);
		TVfi.setText(strFi);
		
		strTheta=String.format("theta: %.3f°\nrel.theta: %.3f°\nsp.theta: %.3f°\n", attitude.theta*R2DG, relativeAttitude.theta*R2DG, setPoint.pitch*R2DG);
		TVtheta.setText(strTheta);		

		strPsi=String.format("x: %.3f\ny: %.3f\nvx: %.3f\nvy: %.3f\npsi: %.3f\nla0: %.3f\nlo0: %.3f\naz: %.3f\nGvx: %.3f\nGvy: %.3f\nGx: %.3f\nGy: %.3f\nSat: %.3f\ndt: %.3f", attitude.x,attitude.y,attitude.vx,attitude.vy,attitude.psi*R2DG,attitude.latitudeZero,attitude.longitudeZero, attitude.az, attitude.xSpeed, attitude.ySpeed,attitude.xPos,attitude.yPos, attitude.nSatellites, attitude.gps_dt);
		TVpsi.setText(strPsi);
    }
    
    private Runnable updateDisplayTask = new Runnable() {
		public void run() {
			updateDisplay();
		}
	};

	
}
