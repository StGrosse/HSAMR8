package parkingRobot.hsamr8;


import lejos.robotics.navigation.Pose;
import lejos.nxt.Battery;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;
//import lejos.nxt.Sound;
import java.lang.Math;
//import lejos.nxt.LCD;

/**
 * Main class for control module
 *
 */
public class ControlRST implements IControl {
	
	int v=0;
	int option = 2; //line follower (1 --- example project, 2 --- PID)
	int esuml =0; //integrator for PID-algo, left (PID Linefollower Version 1)
	int esumr =0; //integrator for PID-algo, right (PID Linefollower Version 1)
	int esum_line =0;//integrator for PID-algo (PID Linefollower version 2)
	double esuml_vw =0; //Fehlersumme für PID v/w-Control, linkes Rad
	double esumr_vw =0; //Fehlersumme für PID v/w-Control, rechtes Rad
	int eoldl = 0; //e(k-1) for PID-algo, left (PID Linefollower Version 1)
	int eoldr =0; //e(k-1) for PID-algo, right (PID Linefollower Version 1)
	int eold = 0; //alter Fehler PID Linefollower Version 2
	double eoldl_vw =0;//alter Fehler PID v/w-Control, linkes Rad
	double eoldr_vw =0;//alter Fehler PID v/w-Control, rechtes Rad
	static final double T_a = 0.100; //sampling time in seconds
	static final int u_r_max =40; //maximale power
	static final int w =100; //Führungsgröße PID (white)
	static final double r_robot = 5.65; //Radius des Roboters (halber Abstand der Räder) in cm
	static final double r_wheel = 2.7; //Radius der Räder in cm
	static final double distPerTurn = 2*Math.PI*r_wheel; //gefahrene Distanz pro Radumdrehung in cm
	static final double distPerDeg = distPerTurn/360; //gefahrene Distanz pro Grad Radumdrehung in cm
	static final double angVelPerPercent = 0; //Winkelgeschwindigkeit pro Prozent Motoransteuerung (Annahme: v~PW)
	static final double offsetAngVelPerPercent = 0; //Offset der Gerade w_wheel(PW)
	static final double kp = 0.0601; //Proportionalbeiwert PID Linefollower
	static final double ki = 0.0082; //Integrierbeiwert PID Linefollower
	static final double kd = 0.095; //Differentierbeiwert PID Linefollower
	boolean straight = true; //Kurve erkannt, fahre bis Kurvenscheitel
	static final int akku_max = 0;//maximale spannung akku in mV
	boolean first = true;
	
	enum dest{
		left, //robot shall drive right
		right, //robot shall drive left
		no, //there's no curve to be driven
		stop //don't drive until the program is restarted --> used for security issues
	}
	dest curve = dest.no;
	//Variablen für calcAngVelPerPercent():
	int encoderSum;
	int encoderSumL=0;
	int encoderSumR=0;
	int time=0;
	int zähler =0;
	int power=10;
	int lock =25;
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
	
    double currentDistance = 0.0;
    double Distance = 0.0;
  
	
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
		
		// MONITOR für PID-Regler Linienverfolgung
		monitor.addNavigationVar("X");
		monitor.addControlVar("RightSensor");
		monitor.addControlVar("LeftSensor");
		monitor.addControlVar("Fehler"); //für Version 2 des PID-Reglers
		monitor.addControlVar("RightMotor");
		monitor.addControlVar("LeftMotor");
		/*MONITOR für Dimensionierung PID-Regler V/W-Control linkes Rad
		monitor.addControlVar("wRadLinks");
		monitor.addControlVar("FehlerLinks");
		monitor.addControlVar("PWLinks");*/
		/*MONITOR für Dimensionierung PID-Regler V/W-Control rechtes Rad
		monitor.addControlVar("wRadRechts");
		monitor.addControlVar("FehlerRechts");
		monitor.addControlVar("PWRechts");*/
		
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
		this.angleMeasurementLeft=this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight=this.encoderRight.getEncoderMeasurement();
		//this.calcAngVelPerPercent();//Methode zur experimentellen Bestimmung von AngVelPerPercent
		
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
	
	// Private methods
	
	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter(){
		setPose(navigation.getPose());
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
	 */
	private void update_LINECTRL_Parameter(){
		if(option == 1){
			this.lineSensorRight		= perception.getRightLineSensor();
			this.lineSensorLeft  		= perception.getLeftLineSensor();
		}
		else if (option ==2){
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
    	/*double kp_l=0;
    	double ki_l=0;
    	double kd_l=0;
    	double kp_r=0;
    	double ki_r=0;
    	double kd_r=0;*/
		double [] speed = this.drive(this.velocity, this.angularVelocity); //berechne benötigte Winkelgeschwindigkeiten
		//Steuerung der Motoren (ohne Regelung) auf Basis von experimentell bestimmter Proportionalitätskonstante
		this.leftMotor.setPower((int)((speed[0]-offsetAngVelPerPercent)/angVelPerPercent)); //mit w_wheel=angVelPerPercent*power+offsetAngVelPerPercent (lineare Näherung)
		this.rightMotor.setPower((int)((speed[1]-offsetAngVelPerPercent)/angVelPerPercent));
		
		/*
		//Berechnung der aktuellen Geschwindigkeit und Winkelgeschwindigkeit aus Daten der Rad-Encoder:
		//Winkelgeschwindigkeit linkes Rad in Grad/sec
		double w_akt_l=this.angleMeasurementLeft.getAngleSum()/this.angleMeasurementLeft.getDeltaT()/1000;
		//Winkelgeschwindigkeit rechtes Rad in Grad/sec
		double w_akt_r=this.angleMeasurementRight.getAngleSum()/this.angleMeasurementRight.getDeltaT()/1000;	
		
		//Regelung der Motoren mit PID-Regler
		double e_l = this.speedDegLeft-w_akt_l;//Fehler links in Grad/sec
		double e_r = this.speedDegRight-w_akt_r;//Fehler rechts in Grad/sec
		
		//Berechnung der Stellgrößen (Pulsweite)
		double u_l = kp_l*e_l+ki_l*T_a*this.esuml_vw+kd_l/T_a*(e_l-this.eoldl_vw);
		double u_r = kp_r*e_r+ki_r*T_a*this.esumr_vw+kd_r/T_a*(e_r-this.eoldr_vw);*/
		
		/* Ausschriften Dimensionierung links
		monitor.writeControlVar("wRadLinks", ""+w_akt_l);
		monitor.writeControlVar("FehlerLinks","" + e_l);
		monitor.writeControlVar("PWLinks","" + u_l);*/
		
		/*Ausschriften Dimensionierung rechts
		monitor.writeControlVar("wRadRechts",""+w_akt_r);
		monitor.writeControlVar("FehlerRechts",""+e_r);
		monitor.writeControlVar("PWRechts",""+u_r);*/
		
		/*//Ansteuerung der Motoren
		leftMotor.setPower((int)u_l);
		rightMotor.setPower((int)u_r);*/
		
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
    		this.setAngularVelocity(15);
    		this.setVelocity(0.1);
    		exec_VWCTRL_ALGO();
    	}
    	else if(option==4){
    		this.calcAngVelPerPercent();
    	}
    	else if(option==5){
    		this.stop();
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
	 * Führungsgröße: 0
	 * Stellgröße: Pulsweite (power)
	 * Berechnung eines gemeinsamen Fehlers für beide Sensoren: Fehler negativ:zu weit rechts, Fehler positiv: zu weit links
	 * gemeinsamer PID-Regler
	 * Interpretation der Reglerausgangsgröße in zwei Stellgrößen
	 * 
	 */
	
	private void exec_LINECTRL_ALGO_opt2(){
		leftMotor.forward();
		rightMotor.forward();
		 //ab 0.13 deutliches Aufschwingen --> erhöhe kd so, dass ausreichend Phasenreserve bei akzeptabler Dynamik
								//optimaler ermittelter Wert auf Gerade:0.09, zerstört aber die Kurvenfahrt aufgrund zu schneller Bewegungen --> wieder etwas veringert 
								//--> schlechtere Dynamik beim Einschwingen, dafür (hoffentlich) stabilere Kurvenfahrt
		
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
		if(lock<=0){		
			if (((e-eold)<=-35) && (this.lineSensorLeft<=50) && (this.lineSensorRight>=50)){
				this.curve=dest.right;
				this.encoderSumL=0;
				this.encoderSumR=0;
				this.straight=true;
				this.stop();
				this.curveAlgo2();
				monitor.writeControlVar("Fehler","" + e);
				monitor.writeControlVar("LeftMotor", "0");
				monitor.writeControlVar("RightMotor","0");
				monitor.writeControlVar("LeftSensor","" + this.lineSensorLeft);
				monitor.writeControlVar("RightSensor","" + this.lineSensorRight);
				return;
			}
	
			if (((e-eold)>=35) && (this.lineSensorLeft>=50) && (this.lineSensorRight<=50)){
				curve=dest.left;
				this.encoderSumL=0;
				this.encoderSumR=0;
				this.straight=true;
				this.stop();
				this.curveAlgo2();
				monitor.writeControlVar("Fehler","" + e);
				monitor.writeControlVar("LeftMotor", "0");
				monitor.writeControlVar("RightMotor","0");
				monitor.writeControlVar("LeftSensor","" + this.lineSensorLeft);
				monitor.writeControlVar("RightSensor","" + this.lineSensorRight);
				return;
			}
		}
				
		if(e<=10)esum_line=0;
		int u_r = (int)(kp*e + ki*esum_line+kd*(e-this.eold));
		int a = u_r_max + u_r;
		int b = u_r_max - u_r;
		/*this.monitor.writeControlComment("u_r_l: "+ a);
		this.monitor.writeControlComment("u_r_r: "+ b);*/
		this.leftMotor.setPower(a);
		this.rightMotor.setPower(b);
		this.esum_line+=e;
		this.eold=e;
		monitor.writeControlVar("Fehler","" + e);
		monitor.writeControlVar("LeftMotor", ""+a);
		monitor.writeControlVar("RightMotor",""+b);
		monitor.writeControlVar("LeftSensor","" + this.lineSensorLeft);
		monitor.writeControlVar("RightSensor","" + this.lineSensorRight);
		lock--;
		}
	}
	/**erster Versuch einer Kurvenfahrt durch Beschreibung eines Bogens, scheitert an der starken Abhängigkeit
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
				monitor.writeControlComment("Kurve zurückgesetzt");
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
				monitor.writeControlComment("Kurve zurückgesetzt");
			}
			else {
				rightMotor.setPower(11);
				leftMotor.setPower(u_r_max+6);
			}
		}
		
	}
	
	/**
	 * Funktion, die eine reibungslose Kurvenfahrt sicherstellen soll, indem sich der Roboter um 90 Grad
	 * um die eigene Achse dreht
	 */
	private void curveAlgo2(){
		this.leftMotor.forward();
		this.rightMotor.forward();
		encoderSumL+=this.angleMeasurementLeft.getAngleSum();//update gefahrener Winkel
		encoderSumR+=this.angleMeasurementRight.getAngleSum();
		double av_encoderSum=0.5*(Math.abs(encoderSumL)+Math.abs(encoderSumR));//Berechnung des Durchschnitts beider Sensoren zur Verbesserung der Genauigkeit
		if( av_encoderSum < 240 && straight){ //Fahrt zum Kurvenscheitel, 200=dist(Sensor,Rad)/(2*pi*r_wheel)*360°
			leftMotor.setPower(u_r_max-15); //maximale Ansteuerung --> schnelles Weiterfahren, Gefahr: schießen über Kurvenscheitel hinaus
			rightMotor.setPower(u_r_max-15);

		}
		else if(av_encoderSum >= 220 && straight){//beenden des Fahrens bis zum Kurvenscheitel
			straight = false;
			encoderSumL =0;
			encoderSumR =0;
			av_encoderSum=0; //Rücksetzen des Durchschnitts, um sofortigen Kurvenabbruch zu verhindern
		}
		if((curve == dest.right) && !straight){		
			if(av_encoderSum>=180) {
				this.curve = dest.no;
				this.esum_line=0;
				this.eold=0;
				monitor.writeControlComment("Kurve zurückgesetzt");
				this.lock = 10;
				this.stop();
			}
			else {
				rightMotor.setPower(u_r_max-15);
				leftMotor.setPower(-u_r_max+15);
			}
		}
	
		if((curve == dest.left) && !straight){
			if(av_encoderSum>=180){
				this.curve = dest.no;
				this.lock = 10;
				this.esum_line=0;
				this.eold=0;
				monitor.writeControlComment("Kurve zurückgesetzt");
				this.stop();
			}
			else {
				rightMotor.setPower(-u_r_max+15);
				leftMotor.setPower(u_r_max-15);
			}	
		}
	}
	private void stop(){
		this.leftMotor.stop();
		this.rightMotor.stop();
	}
		
    /**
     * calculates the left and right angle speed of the both motors with given velocity 
     * and angle velocity of the robot
     * @param v velocity of the robot in m/s
     * @param omega angle velocity of the robot in 1/s???, positiv für mathematisch positive Rotation
     * @return speed array containing the calculated angular velocity of left (0) and right (1) wheel in degree/sec
     */
	private double[] drive(double v, double omega){
		v = v*100;// Umrechnung in cm/s
		double v_l=0; //Geschwindigkeit linkes Rad in cm/s
		double v_r=0; //Geschwindigkeit rechtes Rad in cm/s
		double speedDegLeft=0;
		double speedDegRight=0;
		double speedDegMiddle =v/distPerDeg; //mittlere Winkelgeschwindigkeit der Räder in Grad pro sekunde
		if(omega != 0){//Rotation vorhanden
			double r_m = v/omega; // Radius zum Rotationszentrum in cm
			if(r_m!=0){ //Rotation und Translation
				v_l=(r_m-r_robot)*v/r_m; //berechne Geschwindigkeit des linken Rades in cm/s
				v_r=(r_m+r_robot)*v/r_m;
				
			}
			else{ //nur Rotation
				v_l=-(omega * r_robot);//für math. positive umdrehung muss linkes Rad rückwärts drehen
				v_r=omega*r_robot; //für math. positive umdrehung muss rechtes Rad vorwärts drehen
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
	private void calcAngVelPerPercent(){
		leftMotor.forward();
		rightMotor.forward();
		if(zähler==1){
			monitor.writeControlComment("Akkuspannung: "+lejos.nxt.Battery.getVoltage());
		}
		if (first){
			this.leftMotor.setPower(40);
			this.rightMotor.setPower(40);
			zähler++;
		}
		if(zähler<100 && !first){
			this.leftMotor.setPower(power);
			this.rightMotor.setPower(power);
			this.encoderSumL+=this.angleMeasurementLeft.getAngleSum();
			this.encoderSumR+=this.angleMeasurementRight.getAngleSum();
			this.time+=this.angleMeasurementLeft.getDeltaT();
			zähler++;
		}
		if(zähler==100){
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
			this.zähler=0;
			if(first){
				first=false;
				power=40;
			}
			this.power+=10;
		}
		if(power==50){
			curve=dest.stop;
			this.stop();
		}
		
	}
}
