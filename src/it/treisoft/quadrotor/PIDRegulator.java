package it.treisoft.quadrotor;

public class PIDRegulator 
{
	private float kp;
	private float ki;
	private float kd;
	private float  integrator, smoothingStrength, errorMean,
    previousError, derivative;

	public PIDRegulator(float kp, float ki, float kd, float smoothingStrength)
    {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.smoothingStrength = smoothingStrength;
            previousError = 0.0f;
            
            integrator = 0.0f;
            errorMean = 0.0f;
    }
	
    public float getOutput(float targetAngle, float currentAngle,float gyro, float dt)
    {

            float error = targetAngle - currentAngle;                       
           
            float output = 0.0f;
            
            // Proportional part.
            output += error * kp;
            
            // Integral part.
            integrator += error * ki * dt;
            output += integrator;
            
            // Derivative part, with low pass filter on derivative.     
//            errorMean = errorMean * smoothingStrength
//                                              + error * (1-smoothingStrength);
//            derivative = (errorMean - previousError) / dt;
//            previousError = errorMean;
            output += -gyro * kd;         
           
            
            return output;
    }
    
    public void setCoefficients(float kp, float ki, float kd)
    {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
    }
    
    public void resetIntegrator()
    {
            integrator = 0.0f;
    }   
    public float getDerivator()
    {
            return derivative;
    }   
    
	
}
