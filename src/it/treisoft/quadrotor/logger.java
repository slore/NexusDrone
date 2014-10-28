package it.treisoft.quadrotor;

import it.treisoft.quadrotor.Controller.Motors;
import it.treisoft.quadrotor.Controller.SetPoint;
import it.treisoft.quadrotor.imuReading.Attitude;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import android.content.Context;
import android.os.Environment;


public class logger {
	
	private String sdcard = Environment.getExternalStorageDirectory().getPath();
	private BufferedWriter out;	
	private boolean logging,initialized;
	
	public logger()
	{
		logging=false;
		initialized=false;
		
		String path = sdcard+"/quadrotor_logs/datasensor.csv";
        File nameFile = new File(path);
        try {
			out = new BufferedWriter(new FileWriter(nameFile));			
			out.append("fi" + "," + "theta" +","+"psi"+","+"elevation"+","
						+"relativeFi"+","+"relativeTheta"+","+"relativePsi"+","+"relativeElevation" + "," 
						+ "Vz" +","+"sonarRaw"+","+"spFi"+","+"spTheta"+","+"spPsi"+","+"spAltitude"+","
						+"TimeStamp" +","+"\n");
			initialized=true;
			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void startLog()
	{
		logging=true;
	}
	
	public void stoptLog()
	{
		logging=false;
		try {
			out.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void append(Attitude attitude, SetPoint setPoint, Motors motors)
	{
		
	    try {
			out.append(Float.toString(attitude.fi) + "," + Float.toString(attitude.theta) + ","+Float.toString(attitude.psi)+","+Float.toString(attitude.altitude) + ","
						+ Float.toString(setPoint.roll) + "," + Float.toString(setPoint.pitch) + "," + Float.toString(setPoint.yaw) + "," + Float.toString(setPoint.altitude) + "," 
						+ Float.toString(System.nanoTime()) + ","+ "\n" );
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}

}
