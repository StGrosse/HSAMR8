package parkingRobot.hsamr8;


import lejos.robotics.navigation.Pose;
import lejos.nxt.Battery;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import parkingRobot.INavigation;
//import lejos.nxt.Sound;
import java.lang.Math;
//import lejos.nxt.LCD;
import java.util.LinkedList;

/**
 * Main class for control module
 *
 */
public class ControlRST implements IControl {
	/** 
	 * manual choose of control algorithm mode (different versions)
	 * {@value}1 --- LineFollower extended example project
	 * {@value}2 --- LineFollower PID
	 * {@value}3 --- VW-Control
	 * {@value}4 --- calcAngVelPerPercent
	 * {@value}5 --- stop
	 * {@value}6 --- Beispielsequenz 1.Verteidigung
	 */
	int option = 2; 		
	
	//general constants:
	static final int u_r_max =40; 							//power maximum
	static final double r_robot = 5.65; 					//radius of the robot in cm
	static final double r_wheel = 2.7; 						//radius of the wheels in cm
	static final double distPerTurn = 2*Math.PI*r_wheel; 	//distance per wheel turn in cm
	static final double distPerDeg = distPerTurn/360; 		//distance per degree in cm
	static final double angVelPerPercentR = 8.7; 			//slope of v=f(power) (left wheel)@u=8V
	static final double angVelPerPercentL = 8.5;			//slope of v=f(power) (right wheel)@u=8V
	static final double offsetAngVelPerPercentL = -80.0; 	//offset of v=f(power) (left wheel)@u=8V
	static final double offsetAngVelPerPercentR = -80.0; 	//offset of v=f(power) (right wheel)@u=8V
	//static final int akku_max = 0;							//maximum voltage of akku in mV
	
	//parameter for exec_LINECTRL_ALGO_opt2
	static final double kp = 0.003; //Proportionalbeiwert PID Linefollower absolut:0.0601, neu:0.004
	static final double ki = 0.000; //Integrierbeiwert PID Linefollower absolut:0.0082, neu:0.000
	static final double kd = 0.02; //Differenzierbeiwert PID Linefollower absolut:0.095, neu.0.04
	
	//global variables for exec_LINECTRL_ALGO_opt2
	int v=0;	
	int esum_line =0;				//error-sum for integrator needed in exec_LINECTRL_ALGO_opt2
	int eold = 0; 					//e(k-1) needed in exec_LINECTRL_ALGO_opt2
	
	//global variables for curveAlgo2()
	boolean straight = true; 		//cuve detected --> drive to top of curve
	enum dest{
		left, //robot shall drive right
		right, //robot shall drive left
		no, //there's no curve to be driven
		stop //don't drive until the program is restarted --> used for security issues
	}
	dest curve = dest.no;
	
	//global variables for exec_VWCTRL_ALGO
	double esuml_vw =0; 	//error-sum of left wheel needed in exec_VWCTRL_ALGO
	double esumr_vw =0; 	//error-sum of right wheel needed in exec_VWCTRL_ALGO	
	double eoldl_vw =0;		//e(k-1) of left wheel needed in exec_VWCTRL_ALGO
	double eoldr_vw =0;		//e(k-1) of right wheel needed in exec_VWCTRL_ALGO	
	int u_old_l=0;			//last power of left motor
	int u_old_r=0;			//last power of right motor
	LinkedList<Double> fehler = new LinkedList<>(); //list of last errors in order to calculate floating average
	LinkedList<Double> geschw = new LinkedList<>(); //list of last powers in order to calculate floating average
	boolean newVW=true;
	
	//global variables for example sequence:
	int phase=1;			//phase of example sequence
	int encoderSumSeqL=0;	//driven angle since last reset (left wheel)
	int encoderSumSeqR=0;	//driven angle since last reset (right wheel)
	int kurven=0;			//amount of curves driven
	
	
	//Variablen f黵 calcAngVelPerPercent():(function only used for testing issues)
	boolean first = true;
	int encoderSum;
	int encoderSumL=0;
	int encoderSumR=0;
	int time=0;
	int z鋒ler =0;
	int power=10;
	int lock =25;
	
	boolean nearCurve=true;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft							=	null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderRight							=	null;
	
	/**
	 * reference to data class for measurement of the left wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft 	= 	null;
	/**
	 * reference to data class for measurement of the right wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight	= 	null;
	
	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
	NXTMotor leftMotor = null;
	NXTMotor rightMotor = null;
	
	IPerception perception = null;
	INavigation navigation = null;
	IMonitor monitor = null;
	ControlThread ctrlThread = null;

    int leftMotorPower = 0;
	int rightMotorPower = 0;
	
	double velocity = 0.0;
	double angularVelocity = 0.0;
	
	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();
	
	ControlMode currentCTRLMODE = null;
	
	EncoderSensor controlRightEncoder    = null;
	EncoderSensor controlLeftEncoder     = null;

	int lastTime = 0;
	
    //double currentDistance = 0.0;
    //double Distance = 0.0;
  
	
	/**
	 * provides the reference transfer so that the class knows its corresponding navigation object (to obtain the current 
	 * position of the car from) and starts the control thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param navigation corresponding main module Navigation class object
	 * @param monitor corresponding main module Monitor class object
	 * @param leftMotor corresponding NXTMotor object
	 * @param rightMotor corresponding NXTMotor object
	 */
	public ControlRST(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor, IMonitor monitor){
		//for(int i=0;i<=9;i++)fehler.add(0.0);
		//for(int i=0;i<=9;i++)geschw.add(0.0);
		
		this.perception = perception;
        this.navigation = navigation;
        this.monitor = monitor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		this.currentCTRLMODE = ControlMode.INACTIVE;
			
		this.encoderLeft  = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
		// MONITOR f黵 PID-Regler Linienverfolgung
		/*monitor.addControlVar("RightSensor");
		monitor.addControlVar("LeftSensor");
		monitor.addControlVar("Fehler"); //f黵 Version 2 des PID-Reglers
		monitor.addControlVar("RightMotor");
		monitor.addControlVar("LeftMotor");*/
		//MONITOR f黵 Dimensionierung PID-Regler V/W-Control linkes Rad
		monitor.addControlVar("wRadLinks");
		//monitor.addControlVar("FehlerLinks");
		//monitor.addControlVar("PWLinks");*/
		//MONITOR f黵 Dimensionierung PID-Regler V/W-Control rechtes Rad
		monitor.addControlVar("wRadRechts");
		//monitor.addControlVar("FehlerRechts");
		//monitor.addControlVar("PWRechts");
		
		this.ctrlThread = new ControlThread(this);
		
		ctrlThread.setPriority(Thread.MAX_PRIORITY - 1);
		ctrlThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		ctrlThread.start();
	}
	

	// Inputs
	
	/**
	 * set velocity
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	/**
	 * set angular velocity
	 * @see parkingRobot.IControl#setAngularVelocity(double angularVelocity)
	 */
	public void setAngularVelocity(double angularVelocity) {
		this.angularVelocity = angularVelocity;

	}
	
	/**
	 * set destination
	 * @see parkingRobot.IControl#setDestination(double heading, double x, double y)
	 */
	public void setDestination(double heading, double x, double y){
		this.destination.setHeading((float) heading);
		this.destination.setLocation((float) x, (float) y);
	}
	

	
	/**
	 * sets current pose
	 * @see parkingRobot.IControl#setPose(Pose currentPosition)
	 */
	public void setPose(Pose currentPosition) {
		// TODO Auto-generated method stub
		this.currentPosition = currentPosition;
	}
	

	/**
	 * set control mode
	 */
	public void setCtrlMode(ControlMode ctrl_mode) {
		this.currentCTRLMODE = ctrl_mode;
	}
		
	/**
	 * set start time
	 */
	public void setStartTime(int startTime){
		this.lastTime = startTime;
	}
	
	/**
	 * selection of control-mode
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO(){
	
		switch (currentCTRLMODE)
		{
		  case LINE_CTRL	: update_LINECTRL_Parameter();
		                      exec_LINECTRL_ALGO();
		                      break;
		  case VW_CTRL		: update_VWCTRL_Parameter();
		   					  exec_VWCTRL_ALGO();
		   					  break; 
		  case SETPOSE      : update_SETPOSE_Parameter();
			  				  exec_SETPOSE_ALGO();
		                      break;
		  case PARK_CTRL	: update_PARKCTRL_Parameter();
		  					  exec_PARKCTRL_ALGO();
		  					  break;		  					  
		  case INACTIVE 	: exec_INACTIVE();
			                  break;
		}

	}
	
	public void innerLoop(){
		update_VWCTRL_Parameter();
		exec_VWCTRL_ALGO();
	}
	
	// Private methods
	
	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter(){
		this.angleMeasurementLeft=this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight=this.encoderRight.getEncoderMeasurement();
		encoderSumSeqL+=this.angleMeasurementLeft.getAngleSum();	//driven angle since last reset (left wheel)
		encoderSumSeqR+=this.angleMeasurementRight.getAngleSum();
		encoderSumL+=this.angleMeasurementLeft.getAngleSum();
		encoderSumR+=this.angleMeasurementRight.getAngleSum();
		//this.monitor.writeControlComment("deltaT:"+this.angleMeasurementLeft.getDeltaT());
	}
	
	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter(){
		//Aufgabe 3.4
	}

	/**
	 * update parameters during LINE Control Mode
	 * uses {@link}perception.getRightLineSensor() if {@link}option =1
	 * uses {@link}perception.getRightLineSensorValue() if{@link}option =2 or 6
	 */
	private void update_LINECTRL_Parameter(){
		this.currentPosition=navigation.getPose();
		if(option == 1){
			this.lineSensorRight		= perception.getRightLineSensor();
			this.lineSensorLeft  		= perception.getLeftLineSensor();
		}
		else if (option ==2 || option==6){
			int LLSV = perception.getLeftLineSensorValue();
			int RLSV = perception.getRightLineSensorValue();
			
			if(LLSV>100) this.lineSensorLeft = 100;
			else if (LLSV<0) this.lineSensorLeft = 0;
			else this.lineSensorLeft = LLSV;
			
			if(RLSV>100) this.lineSensorRight = 100;
			else if(RLSV<0) this.lineSensorRight =0;
			else this.lineSensorRight = RLSV;
		}
		
		
	}
	
	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade during VW Control Mode
	 * optionally one of them could be set to zero for simple test.
	 */
    private void exec_VWCTRL_ALGO(){
    	monitor.writeControlComment("VW aufgerufen "+this.velocity+" "+this.angularVelocity);
    	leftMotor.forward();
    	rightMotor.forward();
    	double kp_l=0.031;
    	double ki_l=0.00075;//bei TA=100: 0.00075
    	double kd_l=0.015;//bei TA=100:0.015
    	double kp_r=0.031;
    	double ki_r=0.00075;
    	double kd_r=0.016;//bei TA=100:0.016
    	
		double [] speed = this.drive(this.velocity, this.angularVelocity); //berechne ben鰐igte Winkelgeschwindigkeiten
		this.monitor.writeControlComment("Zielgeschwindigkeit: "+speed[1]);
		int steuerL;
		int steuerR;
		//Steuerung der Motoren (ohne Regelung) auf Basis von experimentell bestimmter Proportionalit鋞skonstante
		if(curve!=dest.no){
		steuerL = (int)(Math.signum(speed[0])*(Math.abs(speed[0])-offsetAngVelPerPercentL)/angVelPerPercentL); //mit w_wheel=angVelPerPercent*power+offsetAngVelPerPercent (lineare N鋒erung)
		steuerR = (int)(Math.signum(speed[1])*(Math.abs(speed[1])-offsetAngVelPerPercentR)/angVelPerPercentR);
		}
		//this.monitor.writeControlComment("links: "+(int)(Math.signum(speed[0])*(Math.abs(speed[0])-offsetAngVelPerPercentL)/angVelPerPercentL)+" rechts: "+(int)(Math.signum(speed[1])*(Math.abs(speed[1])-offsetAngVelPerPercentR)/angVelPerPercentR));
		else{
			steuerL=10;
			steuerR=10;
		}
		
		if(this.newVW){
			u_old_l=steuerL;
			u_old_r=steuerR;
			this.newVW=false;
		}	
		//Berechnung der aktuellen Geschwindigkeit und Winkelgeschwindigkeit aus Daten der Rad-Encoder:
		//Winkelgeschwindigkeit linkes Rad in Grad/sec
		double w_akt_l=(double)(this.angleMeasurementLeft.getAngleSum())/(double)(this.angleMeasurementLeft.getDeltaT()/1000.0);
		//Winkelgeschwindigkeit rechtes Rad in Grad/sec
		double w_akt_r=(double)(this.angleMeasurementRight.getAngleSum())/(double)(this.angleMeasurementRight.getDeltaT()/1000.0);	
		//monitor.writeControlComment("deltaT:"+this.angleMeasurementLeft.getDeltaT());
		
		//Regelung der Motoren mit PID-Regler
		double e_l = speed[0]-w_akt_l;//Fehler links in Grad/sec
		double e_r = speed[1]-w_akt_r;//Fehler rechts in Grad/sec
		esuml_vw+=e_l;
		esumr_vw+=e_r;
		//if(e_l<5)esuml_vw=0;
		//if(e_r<5)esumr_vw=0;
		//Berechnung der Stellgr鲞en (Pulsweite)
		double u_l = kp_l*e_l+ki_l*this.esuml_vw+kd_l*(e_l-this.eoldl_vw);
		double u_r = kp_r*e_r+ki_r*this.esumr_vw+kd_r*(e_r-this.eoldr_vw);
		double pw_l=Math.abs(u_old_l+u_l)>80?80:u_old_l+u_l;
		double pw_r=Math.abs(u_old_r+u_r)>80?80:u_old_r+u_r;
		u_old_l=(int)pw_l;
		u_old_r=(int)pw_r;
		//fehler.add(0, e_r);
		//geschw.add(0,w_akt_r);
		//geschw.remove(geschw.size()-1);
		//fehler.remove(fehler.size()-1);
		//double mw=0;
		//double mw2=0;
		//for(double a:fehler)mw+=a;
		//for(double b:geschw)mw2+=b;
		//mw/=(fehler.size());
		//mw2/=(geschw.size());
		
		this.eoldl_vw=e_l;
		this.eoldr_vw=e_r;
		
		//Ausschriften Dimensionierung links
		monitor.writeControlVar("wRadLinks", ""+w_akt_l);
		//monitor.writeControlVar("FehlerLinks","" + e_l);
		//monitor.writeControlVar("PWLinks","" + (int)pw_l);
		
		//Ausschriften Dimensionierung rechts
		monitor.writeControlVar("wRadRechts",""+w_akt_r);
		//monitor.writeControlVar("FehlerRechts",""+e_r);
		//monitor.writeControlVar("PWRechts",""+(int)pw_r);
		
		//Ansteuerung der Motoren
		monitor.writeControlComment("pulsweite links:"+pw_l);
		leftMotor.setPower((int)pw_l);
		rightMotor.setPower((int)pw_r);
		
	}
	
    private void exec_SETPOSE_ALGO(){
    	//Aufgabe 3.3
	}
	
	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO(){
		//Aufgabe 3.4
	}
	
    private void exec_INACTIVE(){
    	this.stop();
	}
	
	/**
	 * DRIVING along black line
	 * decides which option is executed
	 */
    private void exec_LINECTRL_ALGO(){
    	if(option == 1) exec_LINECTRL_ALGO_opt1(); //Variante 1 der Linienverfolgung
    	else if (option == 2) exec_LINECTRL_ALGO_opt2(); //Variante 2 der Linienverfolgung
    	//Umgehung des Guidance-Moduls zum Testen von v/w-Control --> manuelles Setzen von v und omega --> Sprungantwort
    	else if (option == 3) {
    		this.setAngularVelocity(Math.PI/2); //winkelgeschwindigkeit in 1/s
    		this.setVelocity(0.0); //Geschwindigkeit in m/s
    		//exec_VWCTRL_ALGO();    		
    	}
    	else if(option==4){
    		this.calcAngVelPerPercent();
    	}
    	else if(option==5){
    		this.stop();
    	}
    	else if(option==6){
    		if(this.phase==1){
    			//this.encoderSumSeqL+=this.angleMeasurementLeft.getAngleSum();
    			//this.encoderSumSeqR+=this.angleMeasurementRight.getAngleSum();
    			this.setVelocity(0.1);
    			this.setAngularVelocity(0.0);
    			//this.exec_VWCTRL_ALGO();
    			if((((double)encoderSumSeqL+ (double)encoderSumSeqR )/2*distPerDeg)>=150){
    				this.phase=2;
    				this.encoderSumSeqL=0;
    				this.encoderSumSeqR=0;
    				this.resetVW();
    				this.stop();
    				return;
    			}   			
    		}
    		else if(this.phase==2){
    			//this.encoderSumSeqL+=this.angleMeasurementLeft.getAngleSum();
    			//this.encoderSumSeqR+=this.angleMeasurementRight.getAngleSum();
    			this.setVelocity(0.0);
    			this.setAngularVelocity(Math.PI/12);
    			//this.exec_VWCTRL_ALGO();
    			if(((Math.abs((double)encoderSumSeqL)+(double)encoderSumSeqR)/2)>=r_robot/r_wheel*90){
    				this.phase=3;
    				this.encoderSumSeqL=0;
    				this.encoderSumSeqR=0;
    				this.resetVW();
    				this.stop();
    			}
    		}
    		else if(this.phase==3){
    			//this.encoderSumSeqL+=this.angleMeasurementLeft.getAngleSum();
    			//this.encoderSumSeqR+=this.angleMeasurementRight.getAngleSum();
    			this.setVelocity(0.05);
    			this.setAngularVelocity(0.0);
    			//this.exec_VWCTRL_ALGO();
    			if((((double)encoderSumSeqL+ (double)encoderSumSeqR )/2*distPerDeg)>=30){
    				this.phase=4;
    				this.encoderSumSeqL=0;
    				this.encoderSumSeqR=0;
    				this.resetVW();
    				this.stop();
    			}
    		}
    		else if(this.phase==4){
    			//this.encoderSumSeqL+=this.angleMeasurementLeft.getAngleSum();
    			//this.encoderSumSeqR+=this.angleMeasurementRight.getAngleSum();
    			this.setVelocity(0.0);
    			this.setAngularVelocity(Math.PI/6);
    			//this.exec_VWCTRL_ALGO();
    			if(((Math.abs((double)encoderSumSeqL)+(double)encoderSumSeqR)/2)>=r_robot/r_wheel*90){
    				this.phase=5;
    				//this.option=2;
    				this.encoderSumSeqL=0;
    				this.encoderSumSeqR=0;
    				this.resetVW();
    				this.stop();
    				this.lock=20;
    			}
    		}
    		else if(this.phase==5){
    			this.monitor.writeControlComment("kurven: "+this.kurven);
    			if (this.kurven==4){
    				this.curve=dest.stop;
    			}
    			this.update_LINECTRL_Parameter();
    			this.exec_LINECTRL_ALGO_opt2();
    			
    		}
    	}
    }
    /**
	 * DRIVING along black line
	 * verbessertes Minimalbeispiel
	 * Linienverfolgung fuer gegebene Werte 0,1,2
	 * white = 0, black = 2, grey = 1
	 */
	private void exec_LINECTRL_ALGO_opt1(){
		
		leftMotor.forward();
		rightMotor.forward();
		int lowPower = 3;
		int midPower = 10; //Verbesserung!
		int highPower = 30;
		int white = 0;
		int grey = 1;
		int black = 2;
		
		// MONITOR (example)
		monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
		monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);

        if(this.lineSensorLeft == black && this.lineSensorRight == grey){
			
			// when left sensor is on the line, turn left
    	    leftMotor.setPower(lowPower);
			rightMotor.setPower(highPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn left");
			
		} 
        else if(this.lineSensorRight == black && this.lineSensorLeft == grey){
		
			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(lowPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn right");
		}
		else if(this.lineSensorLeft == black && this.lineSensorRight == white){
			
			// when left sensor is on the line, turn left
			leftMotor.setPower(lowPower);
			rightMotor.setPower(highPower+5);
			
			// MONITOR (example)
			monitor.writeControlComment("turn left");
			
		} 
		else if(this.lineSensorRight == black && this.lineSensorLeft == white){
		
			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(lowPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn right");
		}
		else if(this.lineSensorLeft == grey && this.lineSensorRight == white) {
				
			// when left sensor is on the line, turn left
			leftMotor.setPower(midPower);
			rightMotor.setPower(highPower+5);
			
			// MONITOR (example)
			monitor.writeControlComment("turn left");
				
		} 
		else if(this.lineSensorRight == grey && this.lineSensorLeft == white) {
			
			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(midPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn right");
		}
		else if (this.lineSensorRight == white && this.lineSensorLeft == white) {
			leftMotor.setPower(highPower);
			rightMotor.setPower(highPower);
			
			monitor.writeControlComment("straight on");
		}
		
	}
	
	/**
	 * DRIVING along black line
	 * Linienverfolgung mit PID-Regler (Version 2)
	 * F黨rungsgr鲞e: 0
	 * Stellgr鲞e: Pulsweite (power)
	 * Berechnung eines gemeinsamen Fehlers f黵 beide Sensoren: Fehler negativ:zu weit rechts, Fehler positiv: zu weit links
	 * gemeinsamer PID-Regler
	 * Interpretation der Reglerausgangsgr鲞e in zwei Stellgr鲞en
	 * 
	 */
	
	private void exec_LINECTRL_ALGO_opt2(){
		this.monitor.writeControlComment("Linectrl aufgerufen");
		//leftMotor.forward();
		//rightMotor.forward();
		if(nearCurve() == dest.no){
			this.setVelocity(0.20);
		}
		else{
			this.setVelocity(0.15);
		}
		int e = this.lineSensorLeft - this.lineSensorRight; //Berechne Fehler aus Differenz der Werte --> w = 0
		
		if(curve == dest.stop){
			this.stop();
			return;
		}
		if(curve != dest.no){
			this.curveAlgo2();
			this.monitor.writeControlComment("Kurve");
			return;
		}
		else{
		//Kurvendetektion, funktioniert stellenweise schon gut
		if((nearCurve()!=dest.no) && lock<=0){		
			if (((e-eold)<=-35) && (this.lineSensorLeft<=50) && (this.lineSensorRight>=50)){
				this.curve=nearCurve();
				this.encoderSumL=0;
				this.encoderSumR=0;
				this.straight=true;
				this.resetVW();
				this.curveAlgo2();
				monitor.writeControlVar("Fehler","" + e);
				//monitor.writeControlVar("LeftMotor", "0");
				//monitor.writeControlVar("RightMotor","0");
				monitor.writeControlVar("LeftSensor","" + this.lineSensorLeft);
				monitor.writeControlVar("RightSensor","" + this.lineSensorRight);
				return;
			}
	
			if (((e-eold)>=35) && (this.lineSensorLeft>=50) && (this.lineSensorRight<=50)){
				this.curve=nearCurve();
				this.encoderSumL=0;
				this.encoderSumR=0;
				this.straight=true;
				this.resetVW();
				this.curveAlgo2();
				monitor.writeControlVar("Fehler","" + e);
				//monitor.writeControlVar("LeftMotor", "0");
				//monitor.writeControlVar("RightMotor","0");
				monitor.writeControlVar("LeftSensor","" + this.lineSensorLeft);
				monitor.writeControlVar("RightSensor","" + this.lineSensorRight);
				return;
			}
		}
				
		if(e<=10)esum_line=0;
		double u_r = (kp*e + ki*esum_line+kd*(e-this.eold));
		this.setAngularVelocity(-u_r);
		this.innerLoop();
		//int a = u_r_max + u_r;
		//int b = u_r_max - u_r;
		/*this.monitor.writeControlComment("u_r_l: "+ a);
		this.monitor.writeControlComment("u_r_r: "+ b);*/
		//this.leftMotor.setPower(a);
		//this.rightMotor.setPower(b);
		this.esum_line+=e;
		this.eold=e;
		monitor.writeControlVar("Fehler","" + e);
		//monitor.writeControlVar("LeftMotor", ""+a);
		//monitor.writeControlVar("RightMotor",""+b);
		monitor.writeControlVar("LeftSensor","" + this.lineSensorLeft);
		monitor.writeControlVar("RightSensor","" + this.lineSensorRight);
		lock--;
		}
	}
	/**
	 * erster Versuch einer Kurvenfahrt durch Beschreibung eines Bogens, scheitert an der starken Abh鋘gigkeit
	 * des gefahrenen Radius von Ladezustand des Akkus
	 */
	
	private void curveAlgo(){
		this.leftMotor.forward();
		this.rightMotor.forward();
		if(curve == dest.right){
			this.encoderSum+=this.angleMeasurementRight.getAngleSum();
			if(encoderSum>=388) {//410
				curve = dest.no;
				this.esum_line=0;
				this.eold=0;
				monitor.writeControlComment("Kurve zur點kgesetzt");
				lock = 10;
			}
			else {
				
				rightMotor.setPower(u_r_max+6);//max+6
				leftMotor.setPower(11);//13
			}
		}
		
		if(curve == dest.left){
			this.encoderSum+=this.angleMeasurementLeft.getAngleSum();
			if(encoderSum>=388){
				this.curve = dest.no;
				lock = 10;
				this.esum_line=0;
				this.eold=0;
				monitor.writeControlComment("Kurve zur點kgesetzt");
			}
			else {
				rightMotor.setPower(11);
				leftMotor.setPower(u_r_max+6);
			}
		}
		
	}
	
	/**
	 * Funktion, die eine reibungslose Kurvenfahrt sicherstellen soll, indem der Roboter bis zum Kurvenscheitel f鋒rt und
	 * sich anschlie遝nd um 90 Grad um die eigene Achse dreht
	 */
	private void curveAlgo2(){
		this.leftMotor.forward();
		this.rightMotor.forward();
		//encoderSumL+=this.angleMeasurementLeft.getAngleSum();//update gefahrener Winkel
		//encoderSumR+=this.angleMeasurementRight.getAngleSum();
		double av_encoderSum=0.5*(Math.abs(encoderSumL)+Math.abs(encoderSumR));//Berechnung des Durchschnitts beider Sensoren zur Verbesserung der Genauigkeit
		if( av_encoderSum < 230 && straight){ //Fahrt zum Kurvenscheitel, 200=dist(Sensor,Rad)/(2*pi*r_wheel)*360�
			this.setAngularVelocity(0.0);
			this.setVelocity(0.07);
			this.innerLoop();
			//this.update_VWCTRL_Parameter();
			//this.exec_VWCTRL_ALGO();
			
		}
		else if(av_encoderSum >= 230 && straight){//beenden des Fahrens bis zum Kurvenscheitel
			straight = false;
			this.resetVW();
			encoderSumL =0;
			encoderSumR =0;
			av_encoderSum=0; //R點ksetzen des Durchschnitts, um sofortigen Kurvenabbruch zu verhindern
		}
		if((curve == dest.right) && !straight){		
			if(av_encoderSum>=186) {
				this.curve = dest.no;
				this.esum_line=0;
				this.eold=0;
				this.resetVW();
				monitor.writeControlComment("Kurve zur點kgesetzt");
				this.lock = 10;
				this.stop();
				kurven++;
			}
			else {
				this.setVelocity(0.0);
				this.setAngularVelocity(-Math.PI/5);
				this.innerLoop();
				//this.update_VWCTRL_Parameter();
				//this.exec_VWCTRL_ALGO();
				//rightMotor.setPower((power<(u_r_max-15))?power:(u_r_max-15));
				//leftMotor.setPower(-((power<(u_r_max-15))?power:(u_r_max-15)));
			}
		}
	
		if((curve == dest.left) && !straight){
			if(av_encoderSum>=186){
				this.curve = dest.no;
				this.lock = 10;
				this.esum_line=0;
				this.eold=0;
				this.resetVW();
				monitor.writeControlComment("Kurve zur點kgesetzt");
				this.stop();
				this.kurven++;
			}
			else {
				this.setVelocity(0.0);
				this.setAngularVelocity(Math.PI/5);
				this.innerLoop();
				//this.update_VWCTRL_Parameter();
				//this.exec_VWCTRL_ALGO();
				//power+=1;
				//rightMotor.setPower(-((power<(u_r_max-15))?power:(u_r_max-15)));
				//leftMotor.setPower((power<(u_r_max-15))?power:(u_r_max-15));
			}	
		}
	}
	private void stop(){
		//this.setAngularVelocity(0.0);
		//this.setVelocity(0.0);
		this.leftMotor.stop();
		this.rightMotor.stop();
	}
		
    /**
     * calculates the left and right angle speed of the both motors with given velocity 
     * and angle velocity of the robot
     * @param v velocity of the robot in m/s
     * @param omega angle velocity of the robot in 1/s???, positiv f黵 mathematisch positive Rotation
     * @return speed array containing the calculated angular velocity of left (0) and right (1) wheel in degree/sec
     */
	private double[] drive(double v, double omega){
		v = v*100;// Umrechnung in cm/s
		double v_l=0; //Geschwindigkeit linkes Rad in cm/s
		double v_r=0; //Geschwindigkeit rechtes Rad in cm/s
		double speedDegLeft=0;
		double speedDegRight=0;
		double speedDegMiddle =v/distPerDeg; //mittlere Winkelgeschwindigkeit der R鋎er in Grad pro sekunde
		if(omega != 0){//Rotation vorhanden
			double r_m = v/omega; // Radius zum Rotationszentrum in cm
			if(r_m!=0){ //Rotation und Translation
				v_l=(r_m-r_robot)*v/r_m; //berechne Geschwindigkeit des linken Rades in cm/s
				v_r=(r_m+r_robot)*v/r_m;
				
			}
			else{ //nur Rotation
				v_l=-(omega * r_robot);//f黵 math. positive umdrehung muss linkes Rad r點kw鋜ts drehen
				v_r=omega*r_robot; //f黵 math. positive umdrehung muss rechtes Rad vorw鋜ts drehen
			}
			speedDegLeft  =v_l/distPerDeg; //Winkelgeschwindigkeit linkes Rad in Grad/sec
			speedDegRight =v_r/distPerDeg; //Winkelgeschwindigkeit rechtes Rad in Grad/sec
		}
		else{ //nur Translation
				speedDegLeft=speedDegMiddle;
				speedDegRight=speedDegMiddle;
		}
		double [] speed = {speedDegLeft, speedDegRight};
		return speed;
	}
	/**
	 * Funktion zu Testzwecken, berechnet die Kennlinie w(PW)
	 */
	private void calcAngVelPerPercent(){
		this.update_VWCTRL_Parameter();
		leftMotor.forward();
		rightMotor.forward();
		if(z鋒ler==1){
			monitor.writeControlComment("Akkuspannung: "+lejos.nxt.Battery.getVoltage());
		}
		/*if (first){
			this.leftMotor.setPower(40);
			this.rightMotor.setPower(40);
			z鋒ler++;
		}*/
		if(z鋒ler<100 /*&& !first*/){
			this.leftMotor.setPower(power);
			this.rightMotor.setPower(power);
			this.encoderSumL+=this.angleMeasurementLeft.getAngleSum();
			this.encoderSumR+=this.angleMeasurementRight.getAngleSum();
			this.time+=this.angleMeasurementLeft.getDeltaT();
			z鋒ler++;
		}
		if(z鋒ler==100){
			this.encoderSumL+=this.angleMeasurementLeft.getAngleSum();
			this.encoderSumR+=this.angleMeasurementRight.getAngleSum();
			this.time+=this.angleMeasurementLeft.getDeltaT();
			double angVelL=(double)this.encoderSumL/(double)this.time;
			double angVelR=(double)this.encoderSumR/(double)this.time;
			monitor.writeControlComment(""+this.power+": omega l: "+ angVelL);
			monitor.writeControlComment(""+this.power+": omega r: "+ angVelR);
			this.encoderSumL=0;
			this.encoderSumR=0;
			this.time=0;
			this.z鋒ler=0;
			/*if(first){
				first=false;
				power=80;
			}*/
			this.power+=10;
		}
		if(power==50){
			curve=dest.stop;
			this.stop();
		}
		
	}
	/**
	 * Funktion zum R點ksetzen der Reglerparameter v/w-Control
	 */
	private void resetVW(){
		this.u_old_l=0;
		this.u_old_r=0;
		this.esuml_vw=0;
		this.esumr_vw=0;
		this.eoldl_vw=0;
		this.newVW=true;
		this.eoldr_vw=0;
	}
	
	private dest nearCurve(){
		double x=this.currentPosition.getX();
		double y=this.currentPosition.getY();
		if(y<0.05 && x>1.60){
			monitor.writeControlComment("nah Kurve1: x:"+x+" y:"+y);
			return dest.left;//Kurve 1
		}
		else if(x>1.75 && y>0.4){
			monitor.writeControlComment("nah Kurve2: x:"+x+" y:"+y);
			return dest.left;//Kurve2
		}
		else if(x>1.45 && x<1.7 && y>0.55){
			monitor.writeControlComment("nah Kurve3: x:"+x+" y:"+y);
			return dest.left;
		}
		else if(x>1.45 && x<1.55 && y<0.5 && y>0.25){
			monitor.writeControlComment("nah Kurve4: x:"+x+" y:"+y);
			return dest.right;//Kurve 4
		}
		else if(x<0.5 && x>0.25 && y<0.35 && y>0.25){
			monitor.writeControlComment("nah Kurve5: x:"+x+" y:"+y);
			return dest.right;//Kurve 5
		}
		else if(x<0.35 && x>0.25 && y>0.4){
			monitor.writeControlComment("nah Kurve6: x:"+x+" y:"+y);
			return dest.left;
		}
		else if(x<0.2 && y>0.55){
			monitor.writeControlComment("nah Kurve7: x:"+x+" y:"+y);
			return dest.left;//Kurve7
		}
		else if(x<0.05 && y<0.2){
			monitor.writeControlComment("nah Kurve8: x:"+x+" y:"+y);
			return dest.left;//Kurve8
		}
		else return dest.no;
	}
}
