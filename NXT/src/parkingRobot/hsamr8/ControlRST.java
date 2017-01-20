/* notwendige Änderrungen:
	Entscheidung wo Parklücke
	*/
package parkingRobot.hsamr8;

import lejos.robotics.navigation.Pose;
//import lejos.nxt.Battery;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.Battery;
import lejos.nxt.LCD;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
//import lejos.nxt.Sound;
import parkingRobot.INavigation;
//import lejos.nxt.Sound;
import java.lang.Math;
//import lejos.nxt.LCD;
//import java.util.LinkedList;

/**
 * Main class for control module
 *
 */
public class ControlRST implements IControl {
	/**
	 * manual choose of control algorithm mode (different versions) {@value}1
	 * --- LineFollower extended example project {@value}2 --- LineFollower PID
	 * {@value}3 --- VW-Control {@value}4 --- calcAngVelPerPercent {@value}5 ---
	 * stop {@value}6 --- Beispielsequenz 1.Verteidigung {@value}7 --- test
	 * setpose {@value}8 --- test pathctrl
	 */
	int option = 2;

	// general constants:
	static final int u_r_max = 40; // power maximum
	static final float r_robot = 5.56f; // radius of the robot in cm
	static final float r_wheel = 2.76f; // radius of the wheels in cm
	static final double distPerTurn = 2 * Math.PI * r_wheel; // distance per
																// wheel turn in
																// cm
	static final double distPerDeg = distPerTurn / 360; // distance per degree
														// in cm
	float angVelPerPercent; // slope of v=f(power) (left
													// wheel)@u=8V
	float offsetAngVelPerPercent=0; // offset of
															// v=f(power)
															// (left wheel)@u=8V
	
	// static final int akku_max = 0; //maximum voltage of akku in mV

	// parameter for exec_LINECTRL_ALGO_opt2
	static final float kp_slow = 0.0043f; // Proportionalbeiwert PID 0.0005
	// Linefollower 
	// absolut:
	static final float kp_fast = 0.002f;
	// static final double ki = 0.000; //Integrierbeiwert PID Linefollower
	// absolut:0.0082, neu:0.000
	static final float kd_fast = 0.023f; // Differenzierbeiwert PID Linefollower
	// absolut:0.095, neu.0.025
	static final float kd_slow = 0.032f;// 0.028 0.033
	static final float V_FAST = 0.2f;
	static final float V_SLOW = 0.15f;

	// global variables for exec_LINECTRL_ALGO_opt2
	int v = 0;
	// int esum_line =0; //error-sum for integrator needed in
	// exec_LINECTRL_ALGO_opt2
	int eold = 0; // e(k-1) needed in exec_LINECTRL_ALGO_opt2

	// global variables for curveAlgo2()
	boolean straight = true; // cuve detected --> drive to top of curve

	enum dest {
		left, // robot shall drive right
		right, // robot shall drive left
		no, // there's no curve to be driven
		stop // don't drive --> used for testing
	}

	dest curve = dest.no;

	// global variables for exec_VWCTRL_ALGO
	double esuml_vw = 0; // error-sum of left wheel needed in exec_VWCTRL_ALGO
	double esumr_vw = 0; // error-sum of right wheel needed in exec_VWCTRL_ALGO
	double eoldl_vw = 0; // e(k-1) of left wheel needed in exec_VWCTRL_ALGO
	double eoldr_vw = 0; // e(k-1) of right wheel needed in exec_VWCTRL_ALGO
	int u_old_l = 0; // last power of left motor
	int u_old_r = 0; // last power of right motor
	boolean newVW = true;

	// global variables for example sequence:
	int phase = 1; // phase of example sequence
	int encoderSumSeqL = 0; // driven angle since last reset (left wheel)
	int encoderSumSeqR = 0; // driven angle since last reset (right wheel)
	int kurven = 0; // amount of curves driven

	// Variablen für calcAngVelPerPercent():(function only used for testing
	// issues)
	boolean first = true;
	int encoderSum;
	int encoderSumL = 0;
	int encoderSumR = 0;
	int time = 0;
	int zähler = 0;
	int power = 10;
	int lock = 25;

	boolean nearCurve = true;
	boolean backward =false;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel
	 * which measures the wheels angle difference between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft = null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot
	 * wheel which measures the wheels angle difference between actual an last
	 * request
	 */
	IPerception.EncoderSensor encoderRight = null;

	/**
	 * reference to data class for measurement of the left wheels angle
	 * difference between actual an last request and the corresponding time
	 * difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft = null;
	/**
	 * reference to data class for measurement of the right wheels angle
	 * difference between actual an last request and the corresponding time
	 * difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight = null;

	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on
	 * line border or gray underground, 2 - on line
	 */
	int lineSensorRight = 0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on
	 * line border or gray underground, 2 - on line
	 */
	int lineSensorLeft = 0;

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
	double x3 = 0;
	boolean ParkStatus = false;
	boolean firstSetPose = true;
	int setPosePhase = 1;
	static final float v_sp = 0.07f;
	static final float kp_sp = 20.0f;// 20 10
	static final float kd_sp = 40.0f;// 30 20höherer D-Anteil verhindert
										// Schwingen nicht, verlangsamt es nur
	double eold_sp = 0;
	double strecke = 0;
	double toleranz = 0;
	static final float ABWEICHUNG = 0.01f;

	ControlMode currentCTRLMODE = null;

	EncoderSensor controlRightEncoder = null;
	EncoderSensor controlLeftEncoder = null;

	// ParkControl variables
	float startTime = 0;
	double T = 0;
	float[] path = new float[4];
	double[] x_t = new double[6];
	// double[] y_t=new double[16];
	boolean inv = false;
	boolean firstPark = false;
	double currentTime = 0;// aktuelle Zeit in s
	int line=0;
	enum Slot {
		unten, seite, oben
	}

	Slot currentSlot = Slot.unten;

	// double currentDistance = 0.0;
	// double Distance = 0.0;

	/**
	 * provides the reference transfer so that the class knows its corresponding
	 * navigation object (to obtain the current position of the car from) and
	 * starts the control thread.
	 * 
	 * @param perception
	 *            corresponding main module Perception class object
	 * @param navigation
	 *            corresponding main module Navigation class object
	 * @param monitor
	 *            corresponding main module Monitor class object
	 * @param leftMotor
	 *            corresponding NXTMotor object
	 * @param rightMotor
	 *            corresponding NXTMotor object
	 */
	public ControlRST(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor,
			IMonitor monitor) {
		// for(int i=0;i<=9;i++)fehler.add(0.0);
		// for(int i=0;i<=9;i++)geschw.add(0.0);
		monitor.addControlVar("wZielR");
		float akku=Battery.getVoltage();
		if(akku<=7.3){
			this.angVelPerPercent=16.0f;
			this.offsetAngVelPerPercent=-155.5f;
		}
		else if(akku>7.3 && akku<=7.6){
			this.angVelPerPercent=16.5f;
			this.offsetAngVelPerPercent=-154.7f;
		}
		else if(akku>7.6 && akku<=7.7){
			this.angVelPerPercent=16.9f;
			this.offsetAngVelPerPercent=-156.5f;
		}
		else if(akku>7.7 /*&& akku<=8.3*/){
			this.angVelPerPercent=18.2f;
			this.offsetAngVelPerPercent=-167.4f;
			
		}
		
		this.perception = perception;
		this.navigation = navigation;
		this.monitor = monitor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		this.currentCTRLMODE = ControlMode.INACTIVE;

		this.encoderLeft = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		this.lineSensorRight = perception.getRightLineSensor();
		this.lineSensorLeft = perception.getLeftLineSensor();

		// MONITOR für PID-Regler Linienverfolgung
		/*
		 * monitor.addControlVar("RightSensor");
		 * monitor.addControlVar("LeftSensor"); monitor.addControlVar("Fehler");
		 * //für Version 2 des PID-Reglers monitor.addControlVar("RightMotor");
		 * monitor.addControlVar("LeftMotor");
		 */
		// MONITOR für Dimensionierung PID-Regler V/W-Control linkes Rad
		monitor.addControlVar("wRadLinks");
		monitor.addControlVar("wZielL");
		// monitor.addControlVar("FehlerLinks");
		// monitor.addControlVar("PWLinks");*/
		// MONITOR für Dimensionierung PID-Regler V/W-Control rechtes Rad
		monitor.addControlVar("wRadRechts");
		// monitor.addControlVar("FehlerRechts");
		// monitor.addControlVar("PWRechts");
		// monitor.addControlVar("Fehler setpose");
		// monitor.addControlVar("v");
		// monitor.addControlVar("w");

		monitor.addControlVar("wZielR");
		

		this.ctrlThread = new ControlThread(this);

		ctrlThread.setPriority(Thread.MAX_PRIORITY - 1);
		ctrlThread.setDaemon(true); // background thread that is not need to
									// terminate in order for the user program
									// to terminate
		ctrlThread.start();
	}

	// Inputs

	/**
	 * set velocity
	 * 
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public boolean getParkStatus() {
		return ParkStatus;
	}

	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	/**
	 * set angular velocity
	 * 
	 * @see parkingRobot.IControl#setAngularVelocity(double angularVelocity)
	 */
	public void setAngularVelocity(double angularVelocity) {
		this.angularVelocity = angularVelocity;

	}
	public void setBackward(boolean backward){
		this.backward=backward;
	}

	/**
	 * set destination
	 * 
	 * @see parkingRobot.IControl#setDestination(double heading, double x,
	 *      double y)
	 */
	public void setDestination(double heading, double x, double y) {
		this.destination.setHeading((float) heading);
		this.destination.setLocation((float) x, (float) y);
		this.firstSetPose = true;
		this.setPosePhase = 1;
	}

	/**
	 * sets current pose
	 * 
	 * @see parkingRobot.IControl#setPose(Pose currentPosition)
	 */
	public void setPose(Pose currentPosition) {
		this.currentPosition = currentPosition;
	}

	/**
	 * set path to be driven
	 * 
	 * @param path
	 *            coefficients of polynomial
	 *            y(x)=path[0]+path[1]*x+path[2]*x^2+path[3]*x^3
	 * @param inv
	 *            true if path shall be driven backwards,false otherwise
	 * @param start
	 *            position where path starts @see le
	 *            jos.robotics.navigation.Pose
	 * @param ziel
	 *            position where path ends
	 * @param line line number of the slot
	 * 	 */
	public void setPath(float[] path, boolean inv, Pose start, Pose ziel, int line) {
		this.path = path;
		this.inv = inv;
		this.startPosition = start;
		this.destination = ziel;
		this.firstPark = true;
		this.ParkStatus = false;
		this.line=line;
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
	public void setStartTime(int startTime) {
		this.startTime = startTime / 1000.0f;
		lock=15;
		kurven=0;
	}
	
	public int getAmountOfCurves(){
		return kurven;
	}

	/**
	 * selection of control-mode
	 * 
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO() {

		switch (currentCTRLMODE) {
		case LINE_CTRL:
			update_LINECTRL_Parameter();
			exec_LINECTRL_ALGO();
			break;
		case VW_CTRL:
			update_VWCTRL_Parameter();
			exec_VWCTRL_ALGO();
			break;
		case SETPOSE:
			update_SETPOSE_Parameter();
			exec_SETPOSE_ALGO();
			break;
		case PARK_CTRL:
			update_PARKCTRL_Parameter();
			exec_PARKCTRL_ALGO();
			break;
		case INACTIVE:
			exec_INACTIVE();
			break;
		}

	}

	private void innerLoop() {
		update_VWCTRL_Parameter();
		exec_VWCTRL_ALGO();
	}

	// Private methods

	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter() {
		this.angleMeasurementLeft = this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight = this.encoderRight.getEncoderMeasurement();
		encoderSumSeqL += this.angleMeasurementLeft.getAngleSum(); // driven
																	// angle
																	// since
																	// last
																	// reset
																	// (left
																	// wheel)
		encoderSumSeqR += this.angleMeasurementRight.getAngleSum();
		encoderSumL += this.angleMeasurementLeft.getAngleSum();
		encoderSumR += this.angleMeasurementRight.getAngleSum();
		// this.monitor.writeControlComment("deltaT:"+this.angleMeasurementLeft.getDeltaT());
	}

	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter() {
		setPose(navigation.getPose());
	}

	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter() {
		setPose(this.navigation.getPose());
		this.currentTime = System.currentTimeMillis() / 1000.0;
	}

	/**
	 * update parameters during LINE Control Mode uses
	 * {@link}perception.getRightLineSensor() if {@link}option =1 uses
	 * {@link}perception.getRightLineSensorValue() if{@link}option =2 or 6
	 */
	private void update_LINECTRL_Parameter() {
		this.currentPosition = navigation.getPose();
		if (option == 1) {
			this.lineSensorRight = perception.getRightLineSensor();
			this.lineSensorLeft = perception.getLeftLineSensor();
		} else if (option == 2 || option == 6) {
			int LLSV = perception.getLeftLineSensorValue();
			int RLSV = perception.getRightLineSensorValue();

			if (LLSV > 100)
				this.lineSensorLeft = 100;
			else if (LLSV < 0)
				this.lineSensorLeft = 0;
			else
				this.lineSensorLeft = LLSV;

			if (RLSV > 100)
				this.lineSensorRight = 100;
			else if (RLSV < 0)
				this.lineSensorRight = 0;
			else
				this.lineSensorRight = RLSV;
		}

	}

	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade
	 * during VW Control Mode optionally one of them could be set to zero for
	 * simple test.
	 */
	private void exec_VWCTRL_ALGO() {
		monitor.writeControlComment("VW aufgerufen " + this.velocity + " " + this.angularVelocity);
		leftMotor.forward();
		rightMotor.forward();
		float kp_l = 0.031f;
		float ki_l = 0.00120f;// bei TA=100: 0.00120
		float kd_l = 0.015f;// bei TA=100: 0.082
		float kp_r = 0.031f;
		float ki_r = 0.00120f;
		float kd_r = 0.016f;// bei TA=100: 0.084

		double[] speed = this.drive(this.velocity, this.angularVelocity); // berechne
																			// benötigte
																			// Winkelgeschwindigkeiten
		this.monitor.writeControlComment("Zielgeschwindigkeit: " + speed[1]);
		int steuerL = 0;
		int steuerR = 0;
		// Steuerung der Motoren (ohne Regelung) auf Basis von experimentell
		// bestimmter Proportionalitätskonstante
		if (this.newVW) {
			//monitor.writeControlComment("speed: "+speed[1]);
			if ((this.currentCTRLMODE != ControlMode.LINE_CTRL) /*|| (curve!=dest.no)*/) {
			
				if (speed[0] > 0) {
					steuerL = (int) ((2*Math.abs(speed[0]) - offsetAngVelPerPercent) / angVelPerPercent); // mit
					// w_wheel=angVelPerPercent*power+offsetAngVelPerPercent
				} else if (speed[0] < 0) {
					steuerL = -(int) ((2*Math.abs(speed[0]) - offsetAngVelPerPercent) / angVelPerPercent); // mit

				}
				if (speed[1] > 0) {
					steuerR = (int) ((2*Math.abs(speed[1]) - offsetAngVelPerPercent) / angVelPerPercent);
				} else if (speed[1] < 0) {
					steuerR = -(int) ((2*Math.abs(speed[1]) - offsetAngVelPerPercent) / angVelPerPercent);
				}
				monitor.writeControlComment("Anstieg: "+angVelPerPercent);
			}
			// this.monitor.writeControlComment("links:
			// "+(int)(Math.signum(speed[0])*(Math.abs(speed[0])-offsetAngVelPerPercentL)/angVelPerPercentL)+"
			// rechts:
			// "+(int)(Math.signum(speed[1])*(Math.abs(speed[1])-offsetAngVelPerPercentR)/angVelPerPercentR));
			else {
				steuerL = 9;
				steuerR = 9;
			}

			u_old_l = steuerL;
			u_old_r = steuerR;

		}
		// Berechnung der aktuellen Geschwindigkeit und Winkelgeschwindigkeit
		// aus Daten der Rad-Encoder:
		// Winkelgeschwindigkeit linkes Rad in Grad/sec
		double w_akt_l = (double) (this.angleMeasurementLeft.getAngleSum())
				/ (double) (this.angleMeasurementLeft.getDeltaT() / 1000.0);
		// Winkelgeschwindigkeit rechtes Rad in Grad/sec
		double w_akt_r = (double) (this.angleMeasurementRight.getAngleSum())
				/ (double) (this.angleMeasurementRight.getDeltaT() / 1000.0);
		// monitor.writeControlComment("deltaT:"+this.angleMeasurementLeft.getDeltaT());

		// Regelung der Motoren mit PID-Regler
		double e_l = speed[0] - w_akt_l;// Fehler links in Grad/sec
		double e_r = speed[1] - w_akt_r;// Fehler rechts in Grad/sec

		double pw_l = u_old_l;
		double pw_r = u_old_r;
		if (this.currentCTRLMODE == ControlMode.LINE_CTRL || !this.newVW) {
			esuml_vw += e_l;
			esumr_vw += e_r;
			// if(e_l<5)esuml_vw=0;
			// if(e_r<5)esumr_vw=0;
			// Berechnung der Stellgrößen (Pulsweite)
			double u_l = kp_l * e_l + ki_l * this.esuml_vw + kd_l * (e_l - this.eoldl_vw);
			double u_r = kp_r * e_r + ki_r * this.esumr_vw + kd_r * (e_r - this.eoldr_vw);

			// fehler.add(0, e_r);
			// geschw.add(0,w_akt_r);
			// geschw.remove(geschw.size()-1);
			// fehler.remove(fehler.size()-1);
			// double mw=0;
			// double mw2=0;
			// for(double a:fehler)mw+=a;
			// for(double b:geschw)mw2+=b;
			// mw/=(fehler.size());
			// mw2/=(geschw.size());

			this.eoldl_vw = e_l;
			this.eoldr_vw = e_r;
			pw_l = Math.abs(u_old_l + u_l) > 100 ? 100 : u_old_l + u_l;
			pw_r = Math.abs(u_old_r + u_r) > 100 ? 100 : u_old_r + u_r;
		}
		
		u_old_l = (int) pw_l;
		u_old_r = (int) pw_r;
		// Ausschriften Dimensionierung links
		monitor.writeControlVar("wZielL", "" + speed[0]);
		monitor.writeControlVar("wRadLinks", "" + w_akt_l);
		// monitor.writeControlVar("FehlerLinks","" + e_l);
		// monitor.writeControlVar("PWLinks","" + (int)pw_l);

		// Ausschriften Dimensionierung rechts
		monitor.writeControlVar("wZielR", "" + speed[1]);
		monitor.writeControlVar("wRadRechts", "" + w_akt_r);
		// monitor.writeControlVar("FehlerRechts",""+e_r);
		// monitor.writeControlVar("PWRechts",""+(int)pw_r);

		// Ansteuerung der Motoren
		monitor.writeControlComment("pulsweite links:" + pw_l);
		monitor.writeControlComment("pulsweite rechts:" + pw_r);
		leftMotor.setPower((int) pw_l);
		rightMotor.setPower((int) pw_r);
		this.newVW = false;
//		this.currentCTRLMODE=ControlMode.LINE_CTRL;

	}

	private void exec_SETPOSE_ALGO() {
		double deltax = this.destination.getX() - this.currentPosition.getX();
		this.monitor.writeControlComment("deltax: " + deltax);
		double deltay = this.destination.getY() - this.currentPosition.getY();
		this.monitor.writeControlComment("deltay: " + deltay);

		double currentHeading = this.currentPosition.getHeading();
		while (currentHeading < 0) {
			currentHeading += 2 * Math.PI;
		}
		 this.monitor.writeControlComment("akt Winkel: "+currentHeading);

		if (this.firstSetPose) {
			this.startPosition = this.currentPosition;
			double x1 = this.destination.getX() - this.startPosition.getX();
			double x2 = this.destination.getY() - this.startPosition.getY();
			if (x1 == 0) {
				this.strecke = x2;
				if (x2 > 0)
					this.x3 = Math.PI / 2;
				else if (x2 < 0)
					this.x3 = 3 * Math.PI / 2;
				else
					this.x3 = currentHeading;
			} else if (x2 == 0) {
				this.strecke = x1;
				if (x1 > 0)
					this.x3 = 0;
				else if (x1 < 0)
					this.x3 = Math.PI;
			} else {
				this.strecke = Math.sqrt(x2 * x2 + x1 * x1);
				if (x2 > 0 && x1 > 0)
					this.x3 = Math.atan(x2 / x1);
				else if (x2 < 0 && x1 > 0)
					this.x3 = 2 * Math.PI + Math.atan(x2 / x1);
				else if (x2 > 0 && x1 < 0)
					this.x3 = Math.PI + Math.atan(x2 / x1);
				else if (x2 < 0 && x1 < 0)
					this.x3 = Math.PI + Math.atan(x2 / x1);
			}
			this.monitor.writeControlComment("strecke: " + strecke + " ,Winkel: " + x3);

			this.firstSetPose = false;
			this.ParkStatus = false;
		}
		double deltax3 = currentHeading - this.x3;
		if (Math.abs(deltax3) > Math.PI) {
			if (deltax3 < 0)
				deltax3 += 2 * Math.PI;
			else if (deltax3 > 0)
				deltax3 -= 2 * Math.PI;
		}
		this.monitor.writeControlComment("deltax3: " + deltax3);

		// zuerst drehen in die richtige Richtung
		if (this.setPosePhase == 1) {
			if (Math.abs(deltax3) > Math.PI / 70) {// Annahme:drehen mit 45°/s
													// --> 4.5°/Abtastung -->
													// maximaler Fehler pi/40
				this.setVelocity(0.0);
				if (deltax3 < 0)
					this.setAngularVelocity(+Math.PI / 7);
				else if (deltax3 > 0)
					this.setAngularVelocity(-Math.PI / 7);
				this.innerLoop();
			} else {// nur einmal abgearbeitet
				this.monitor.writeControlComment("Abw. 1. Drehen: deltax3: " + deltax3 + " x: "
						+ this.currentPosition.getX() + " y: " + this.currentPosition.getY());
				this.resetVW();
				this.stop();
				this.toleranz = Math.abs(Math.tan(deltax3) * this.strecke) + ABWEICHUNG; // zulässige
																							// Abweichung
																							// vom
																							// Ziel
																							// abhängig
																							// von
																							// der
																							// Winkelabweichung
				this.monitor.writeControlComment("Toleranz: " + this.toleranz);
				this.setPosePhase = 2;
			}
		}
		// Zielpunkt anfahren
		if (this.setPosePhase == 2) {
			double abstand2 = deltax * deltax + deltay * deltay;
			this.monitor.writeControlComment("abstand^2: " + abstand2);
			if (abstand2 > (this.toleranz * this.toleranz)) {
				double e_sp = deltax3 * v_sp;
				this.monitor.writeControlVar("Fehler setpose", "" + e_sp);
				this.setVelocity(v_sp);
				double w = -(kp_sp * e_sp + kd_sp * (e_sp - this.eold_sp));
				this.setAngularVelocity(w);
				this.innerLoop();
				this.eold_sp = e_sp;

			} else {
				this.setPosePhase = 3;
				this.monitor.writeControlComment("ziel erreicht, quadr. abw: " + abstand2);
				this.stop();
				this.resetVW();
			}
		}
		// Zielpose einnehmen
		if (this.setPosePhase == 3) {
			double deltaphi = currentHeading - this.destination.getHeading();
			if (Math.abs(deltaphi) > Math.PI) {
				if (deltaphi > 0)
					deltaphi -= 2 * Math.PI;
				else if (deltaphi < 0)
					deltaphi += 2 * Math.PI;
			}
			this.monitor.writeControlComment("deltaphi:" + deltaphi);
			if (Math.abs(deltaphi) > Math.PI / 70 && !ParkStatus) {
				this.setVelocity(0.0);
				if (deltaphi < 0)
					this.setAngularVelocity(+Math.PI / 7);
				else if (deltaphi > 0)
					this.setAngularVelocity(-Math.PI / 7);
				this.innerLoop();
			}

			else {
				this.monitor.writeControlComment("Zielpose erreicht, abw: " + deltaphi);
				this.ParkStatus = true;
				this.resetVW();
				this.stop();// testen
			}
		}
	}

	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO() {
		monitor.writeControlComment("parkcontrol aufgerufen");
		// für rückwärts einparken muss v, v0 negiert werden, sonst bleibt die
		// Planung gleich
		// für die Lücke oben muss v0 negiert werden
		// für seitliche Lücke muss omega negiert werden
		if (this.firstPark) {
			if (line==1) {
				this.currentSlot = Slot.seite;
				this.monitor.writeControlComment("seite");
			} else if (line==0) {
				this.currentSlot = Slot.unten;
				this.monitor.writeControlComment("unten");
			} else if(line==4){
				this.currentSlot = Slot.oben;
				this.monitor.writeControlComment("oben");
			}

			double v0;
			if ((inv && !(currentSlot == Slot.oben)) || (!inv && (currentSlot== Slot.oben))) {
				v0 = -0.05;
			} else {
				v0 = 0.05;
			}
			monitor.writeControlComment("v0: " + v0);
			float xs = this.startPosition.getX();
			float ys = this.startPosition.getY();
			float xz = this.destination.getX();
			float yz = this.destination.getY();
			this.T = Math.sqrt(Math.pow((xz - xs), 2) + Math.pow((yz - ys), 2)) / (Math.abs(v0) * 1);
			this.x_t[5] = 6 * (-T * v0 - xs + xz) / (Math.pow(T, 5));
			this.x_t[4] = 15 * (T * v0 + xs - xz) / (Math.pow(T, 4));
			this.x_t[3] = 10 * (-T * v0 - xs + xz) / (Math.pow(T, 3));
			this.x_t[2] = 0;
			this.x_t[1] = v0;
			this.x_t[0] = xs;
			this.firstPark = false;
			monitor.writeControlComment("Parkcontrol initialisiert, T: " + T);
			this.setStartTime((int) System.currentTimeMillis());
			this.resetVW();
			this.stop();
			this.monitor.writeControlComment("pfad:" + path[0] + path[1] + path[2] + path[3]);
			return;
		}
		LCD.clear();
		LCD.drawString(
				"Z: " + Math.round(this.destination.getX()*100)/100.0f + ", " + Math.round(this.destination.getY()*100)/100.0f + ", " + Math.round(this.destination.getHeading()*100)/100.0f,
				0, 4);
		LCD.drawString("a: " + Math.round(this.currentPosition.getX()*100)/100.0f + ", " + Math.round(this.currentPosition.getY()*100)/100.0f + ", "
				+ Math.round(this.currentPosition.getHeading()*100)/100.0f, 0, 5);
		LCD.drawString(""+this.currentSlot, 0, 2);
//		LCD.drawString(""+T, 0, 3);
		LCD.drawString(""+path[0], 0, 0);
		LCD.drawString(""+path[1], 0, 1);
		LCD.drawString(""+path[2], 0, 2);
		LCD.drawString(""+path[3], 0, 3);
		//LCD.drawString(""+Math.round(this.destination.getX()*100)/100.0f+" "+Math.round(this.destination.getY()*100)/100.0f, 0, 4);
		//LCD.drawString(""+Math.round(this.startPosition.getX()*100)/100.0f+" "+Math.round(this.startPosition.getY()*100)/100.0f, 0, 4);

		
		double t = this.currentTime - this.startTime;
		monitor.writeControlComment("t: " + t);
		if ((T - t) < 0.0) {
			double currentHeading = this.currentPosition.getHeading();
			while (currentHeading < 0) {
				currentHeading += 2 * Math.PI;
			}
			double deltaphi = currentHeading - this.destination.getHeading();
			if (Math.abs(deltaphi) > Math.PI) {
				if (deltaphi > 0)
					deltaphi -= 2 * Math.PI;
				else if (deltaphi < 0)
					deltaphi += 2 * Math.PI;
			}
			this.monitor.writeControlComment("deltaphi:" + deltaphi);
			if (Math.abs(deltaphi) > Math.PI / 120 && !ParkStatus) {
				this.setVelocity(0.0);
				if (deltaphi < 0)
					this.setAngularVelocity(+Math.PI / 6);
				else if (deltaphi > 0)
					this.setAngularVelocity(-Math.PI / 6);
				this.innerLoop();
			}

			else {
				this.monitor.writeControlComment("Ziel erreicht, abw: phi: " + deltaphi + " x: "
						+ this.currentPosition.getX() + " y: " + (this.currentPosition.getY()));
				this.ParkStatus = true;
				this.resetVW();
				this.stop();// testen
				//this.currentCTRLMODE = ControlMode.LINE_CTRL;
				this.lock=20;
			}
			return;
		}
		double x = 0;
		double dx = 0;
		double d2x = 0;
		double y = 0;
		double dy_x = 0;

		for (int j = 0; j < this.x_t.length; j++) {// berechne Wert x(t)
			x += this.x_t[j] * Math.pow(t, j);
		}
		/*
		 * for(int k=1; k<this.x_t.length-1;k++){//berechne Wert von x'(t)
		 * dx+=k*this.x_t[k]*Math.pow(t, k-1); }
		 */
		dx = 5 * x_t[5] * Math.pow(t, 4) + 4 * x_t[4] * Math.pow(t, 3) + 3 * x_t[3] * Math.pow(t, 2) + 2 * x_t[2] * t
				+ x_t[1];
		/*
		 * for(int l=2; l<this.x_t.length-2;l++){//berechne Wert von x''(t)
		 * d2x+=l*(l-1)*x_t[l]*Math.pow(t, l-2); }
		 */
		d2x = 20 * x_t[5] * Math.pow(t, 3) + 12 * x_t[4] * Math.pow(t, 2) + 6 * x_t[3] * t + 2 * x_t[2];
		/*
		 * for(int i=0; i<this.path.length;i++){//berechne Wert von y(t)
		 * y+=path[i]*Math.pow(x,i); }
		 */
		dy_x = 3 * path[3] * Math.pow(x, 2) + 2 * path[2] * x + path[1];
		double d2y_x = 6 * this.path[3] * x + 2 * this.path[2];
		double dy_t = dy_x * dx;
		double d2y_t = d2y_x * Math.pow(dx, 2) + dy_x * d2x;// Regel von Faa die
															// Bruno
		this.monitor.writeControlComment("dx: " + dx + " dy_t:" + dy_t);
		double v;
		if (this.inv) {
			v = -Math.sqrt(dx * dx + dy_t * dy_t);
		} else {
			v = Math.sqrt(dx * dx + dy_t * dy_t);
		}

		monitor.writeControlVar("v", "" + v);
		this.setVelocity(v);
		double w = 1 / (v * v) * (dx * d2y_t - d2x * dy_t);
		if (currentSlot == Slot.seite /*&& !inv*/) {
			w *= (-1);
		}

		this.setAngularVelocity(w);
		monitor.writeControlVar("w", "" + w);
		this.innerLoop();
	}

	private void exec_INACTIVE() {
		this.stop();
	}

	/**
	 * DRIVING along black line decides which option is executed
	 */
	private void exec_LINECTRL_ALGO() {
		if (option == 1)
			exec_LINECTRL_ALGO_opt1(); // Variante 1 der Linienverfolgung
		else if (option == 2)
			exec_LINECTRL_ALGO_opt2(); // Variante 2 der Linienverfolgung
		// Umgehung des Guidance-Moduls zum Testen von v/w-Control --> manuelles
		// Setzen von v und omega --> Sprungantwort
		else if (option == 3) {
			this.setAngularVelocity(Math.PI / 2); // winkelgeschwindigkeit in
													// 1/s
			this.setVelocity(0.0); // Geschwindigkeit in m/s
			// exec_VWCTRL_ALGO();
		} else if (option == 4) {
			this.calcAngVelPerPercent();
		} else if (option == 5) {
			this.stop();
		} else if (option == 6) {
			if (this.phase == 1) {
				// this.encoderSumSeqL+=this.angleMeasurementLeft.getAngleSum();
				// this.encoderSumSeqR+=this.angleMeasurementRight.getAngleSum();
				this.setVelocity(0.1);
				this.setAngularVelocity(0.0);
				// this.exec_VWCTRL_ALGO();
				if ((((double) encoderSumSeqL + (double) encoderSumSeqR) / 2 * distPerDeg) >= 150) {
					this.phase = 2;
					this.encoderSumSeqL = 0;
					this.encoderSumSeqR = 0;
					this.resetVW();
					this.stop();
					return;
				}
			} else if (this.phase == 2) {
				// this.encoderSumSeqL+=this.angleMeasurementLeft.getAngleSum();
				// this.encoderSumSeqR+=this.angleMeasurementRight.getAngleSum();
				this.setVelocity(0.0);
				this.setAngularVelocity(Math.PI / 12);
				// this.exec_VWCTRL_ALGO();
				if (((Math.abs((double) encoderSumSeqL) + (double) encoderSumSeqR) / 2) >= r_robot / r_wheel * 90) {
					this.phase = 3;
					this.encoderSumSeqL = 0;
					this.encoderSumSeqR = 0;
					this.resetVW();
					this.stop();
				}
			} else if (this.phase == 3) {
				// this.encoderSumSeqL+=this.angleMeasurementLeft.getAngleSum();
				// this.encoderSumSeqR+=this.angleMeasurementRight.getAngleSum();
				this.setVelocity(0.05);
				this.setAngularVelocity(0.0);
				// this.exec_VWCTRL_ALGO();
				if ((((double) encoderSumSeqL + (double) encoderSumSeqR) / 2 * distPerDeg) >= 30) {
					this.phase = 4;
					this.encoderSumSeqL = 0;
					this.encoderSumSeqR = 0;
					this.resetVW();
					this.stop();
				}
			} else if (this.phase == 4) {
				// this.encoderSumSeqL+=this.angleMeasurementLeft.getAngleSum();
				// this.encoderSumSeqR+=this.angleMeasurementRight.getAngleSum();
				this.setVelocity(0.0);
				this.setAngularVelocity(Math.PI / 6);
				// this.exec_VWCTRL_ALGO();
				if (((Math.abs((double) encoderSumSeqL) + (double) encoderSumSeqR) / 2) >= r_robot / r_wheel * 90) {
					this.phase = 5;
					// this.option=2;
					this.encoderSumSeqL = 0;
					this.encoderSumSeqR = 0;
					this.resetVW();
					this.stop();
					this.lock = 20;
				}
			} else if (this.phase == 5) {
				this.monitor.writeControlComment("kurven: " + this.kurven);
				if (this.kurven == 4) {
					this.curve = dest.stop;
				}
				this.update_LINECTRL_Parameter();
				this.exec_LINECTRL_ALGO_opt2();

			}

		}
		if (option == 7) {
			this.destination.setLocation((float) 1.5, (float) 0.0);
			this.destination.setHeading((float) 0.0);
			this.update_SETPOSE_Parameter();
			this.exec_SETPOSE_ALGO();
		}
		if (option == 8) {
			if (phase == 1 && !ParkStatus) {
				this.destination.setLocation(0.15f, 0.0f);
				this.destination.setHeading(0.0f);
				this.update_SETPOSE_Parameter();
				this.exec_SETPOSE_ALGO();
			} else if (phase == 1 && ParkStatus) {
				this.stop();
				// this.resetVW();
				this.currentCTRLMODE = ControlMode.PARK_CTRL;
				phase = 2;
				ParkStatus = false;
				monitor.writeControlComment("Ende Phase 1");

				float[] a = { -0.1103703703703716f, 1.8962962962963157f, -7.901234567901323f, 7.023319615912314f };
				this.setPath(a, false, new Pose(0.15f, 0.02f, 0.0f), new Pose(0.6f, -0.3f, 0.0f),0);
				return;

			}
			if (phase == 2 && !ParkStatus) {
				monitor.writeControlComment("Start Phase2");
				this.update_PARKCTRL_Parameter();
				this.exec_PARKCTRL_ALGO();
			} else if (phase == 2 && ParkStatus) {
				this.stop();
				this.resetVW();
				ParkStatus = false;
				phase = 3;
				this.currentCTRLMODE = ControlMode.PARK_CTRL;
				float[] a = { -0.1103703703703716f, 1.8962962962963157f, -7.901234567901323f, 7.023319615912314f };
				this.setPath(a, true, new Pose(0.6f, -0.3f, 0.0f), new Pose(0.15f, 0.02f, 0.0f),0);
				return;

			}

			if (phase == 3 && !ParkStatus) {
				monitor.writeControlComment("Start Phase3");
				this.update_PARKCTRL_Parameter();
				this.exec_PARKCTRL_ALGO();
			} else if (phase == 3 && ParkStatus) {
				this.stop();
			}

		}
	}

	/**
	 * DRIVING along black line verbessertes Minimalbeispiel Linienverfolgung
	 * fuer gegebene Werte 0,1,2 white = 0, black = 2, grey = 1
	 */
	private void exec_LINECTRL_ALGO_opt1() {

		leftMotor.forward();
		rightMotor.forward();
		int lowPower = 3;
		int midPower = 10; // Verbesserung!
		int highPower = 30;
		int white = 0;
		int grey = 1;
		int black = 2;

		// MONITOR (example)
		monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
		monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);

		if (this.lineSensorLeft == black && this.lineSensorRight == grey) {

			// when left sensor is on the line, turn left
			leftMotor.setPower(lowPower);
			rightMotor.setPower(highPower);

			// MONITOR (example)
			monitor.writeControlComment("turn left");

		} else if (this.lineSensorRight == black && this.lineSensorLeft == grey) {

			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(lowPower);

			// MONITOR (example)
			monitor.writeControlComment("turn right");
		} else if (this.lineSensorLeft == black && this.lineSensorRight == white) {

			// when left sensor is on the line, turn left
			leftMotor.setPower(lowPower);
			rightMotor.setPower(highPower + 5);

			// MONITOR (example)
			monitor.writeControlComment("turn left");

		} else if (this.lineSensorRight == black && this.lineSensorLeft == white) {

			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(lowPower);

			// MONITOR (example)
			monitor.writeControlComment("turn right");
		} else if (this.lineSensorLeft == grey && this.lineSensorRight == white) {

			// when left sensor is on the line, turn left
			leftMotor.setPower(midPower);
			rightMotor.setPower(highPower + 5);

			// MONITOR (example)
			monitor.writeControlComment("turn left");

		} else if (this.lineSensorRight == grey && this.lineSensorLeft == white) {

			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(midPower);

			// MONITOR (example)
			monitor.writeControlComment("turn right");
		} else if (this.lineSensorRight == white && this.lineSensorLeft == white) {
			leftMotor.setPower(highPower);
			rightMotor.setPower(highPower);

			monitor.writeControlComment("straight on");
		}

	}

	/**
	 * DRIVING along black line Linienverfolgung mit PID-Regler (Version 2)
	 * Führungsgröße: 0 Stellgröße: Pulsweite (power) Berechnung eines
	 * gemeinsamen Fehlers für beide Sensoren: Fehler negativ:zu weit rechts,
	 * Fehler positiv: zu weit links gemeinsamer PID-Regler Interpretation der
	 * Reglerausgangsgröße in zwei Stellgrößen
	 * 
	 */

	private void exec_LINECTRL_ALGO_opt2() {
		LCD.drawString(""+this.currentPosition.getX()+" "+this.currentPosition.getY(), 0, 0);
		this.ParkStatus = false;
		this.monitor.writeControlComment("Linectrl aufgerufen");
		// leftMotor.forward();
		// rightMotor.forward();
		double kd;
		double kp;
		if ((nearCurve() == null) && !nearSlot() && lock<0) {
			this.setVelocity(V_FAST);// eigentlich fast
			kd = kd_fast;
			kp = kp_fast;

		} else {
			this.setVelocity(V_SLOW);
			kd = kd_slow;
			kp = kp_slow;
		}
		int e = this.lineSensorLeft - this.lineSensorRight; // Berechne Fehler
															// aus Differenz der
															// Werte --> w = 0

		if (curve == dest.stop) {
			this.stop();
			return;
		}
		if (curve != dest.no) {
			this.curveAlgo2();
			// this.monitor.writeControlComment("Kurve");
			return;
		} else {
			// Kurvendetektion, funktioniert stellenweise schon gut
			dest a = nearCurve();
			if ((a != dest.no) && (a != null) && lock <= 0) {
				if (a == dest.left && ((e - eold) <= -15)
						&& (this.lineSensorLeft <= 50) /*
														 * &&
														 * (this.lineSensorRight
														 * >=50)
														 */) {
					this.curve = nearCurve();
					this.encoderSumL = 0;
					this.encoderSumR = 0;
					this.straight = true;
					// this.resetVW();
					this.curveAlgo2();
					monitor.writeControlVar("Fehler", "" + e);
					// monitor.writeControlVar("LeftMotor", "0");
					// monitor.writeControlVar("RightMotor","0");
					monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
					monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);
					return;
				}

				if (a == dest.right
						&& ((e - eold) >= 15) /* && (this.lineSensorLeft>=50) */ && (this.lineSensorRight <= 50)) {
					this.curve = nearCurve();
					this.encoderSumL = 0;
					this.encoderSumR = 0;
					this.straight = true;
					// this.resetVW();
					this.curveAlgo2();
					monitor.writeControlVar("Fehler", "" + e);
					// monitor.writeControlVar("LeftMotor", "0");
					// monitor.writeControlVar("RightMotor","0");
					monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
					monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);
					return;
				}
			}

			double u_r = (kp * e + kd * (e - this.eold));
			this.setAngularVelocity(-u_r);
			this.innerLoop();
			// int a = u_r_max + u_r;
			// int b = u_r_max - u_r;
			/*
			 * this.monitor.writeControlComment("u_r_l: "+ a);
			 * this.monitor.writeControlComment("u_r_r: "+ b);
			 */
			// this.leftMotor.setPower(a);
			// this.rightMotor.setPower(b);
			this.eold = e;
			monitor.writeControlVar("Fehler", "" + e);
			// monitor.writeControlVar("LeftMotor", ""+a);
			// monitor.writeControlVar("RightMotor",""+b);
			monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
			monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);
			lock--;
		}
	}

	/**
	 * erster Versuch einer Kurvenfahrt durch Beschreibung eines Bogens,
	 * scheitert an der starken Abhängigkeit des gefahrenen Radius von
	 * Ladezustand des Akkus
	 */

	/**
	 * Funktion, die eine reibungslose Kurvenfahrt sicherstellen soll, indem der
	 * Roboter bis zum Kurvenscheitel fährt und sich anschließend um 90 Grad um
	 * die eigene Achse dreht
	 */
	private void curveAlgo2() {
		this.leftMotor.forward();
		this.rightMotor.forward();
		// encoderSumL+=this.angleMeasurementLeft.getAngleSum();//update
		// gefahrener Winkel
		// encoderSumR+=this.angleMeasurementRight.getAngleSum();
		double av_encoderSum = 0.5 * (Math.abs(encoderSumL) + Math.abs(encoderSumR));// Berechnung
																						// des
																						// Durchschnitts
																						// beider
																						// Sensoren
																						// zur
																						// Verbesserung
																						// der
																						// Genauigkeit
		if (av_encoderSum < 190 && straight) { // Fahrt zum Kurvenscheitel,
												// 200=dist(Sensor,Rad)/(2*pi*r_wheel)*360„1¤7
			this.setAngularVelocity(0.0);
			this.setVelocity(0.1);
			this.innerLoop();
			// this.update_VWCTRL_Parameter();
			// this.exec_VWCTRL_ALGO();

		} else if (av_encoderSum >= 190 && straight) {// beenden des Fahrens bis
														// zum Kurvenscheitel
			straight = false;
			this.resetVW();
			//this.stop();
			encoderSumL = 0;
			encoderSumR = 0;
			av_encoderSum = 0; // Rücksetzen des Durchschnitts, um sofortigen
								// Kurvenabbruch zu verhindern
		}
		if ((curve == dest.right) && !straight) {
			if (av_encoderSum >= 170) {
				this.curve = dest.no;
				// this.esum_line=0;
				this.eold = 0;
				this.resetVW();
				monitor.writeControlComment("Kurve zurückgesetzt");
				this.lock = 10;
				this.stop();
				kurven++;
			} else {
				this.setVelocity(0.0);
				this.setAngularVelocity(-Math.PI / 3);
				this.innerLoop();
				// this.update_VWCTRL_Parameter();
				// this.exec_VWCTRL_ALGO();
				// rightMotor.setPower((power<(u_r_max-15))?power:(u_r_max-15));
				// leftMotor.setPower(-((power<(u_r_max-15))?power:(u_r_max-15)));
			}
		}

		if ((curve == dest.left) && !straight) {
			if (av_encoderSum >= 170) {
				this.curve = dest.no;
				this.lock = 10;
				// this.esum_line=0;
				this.eold = 0;
				this.resetVW();
				monitor.writeControlComment("Kurve zurückgesetzt");
				this.stop();
				this.kurven++;
			} else {
				this.setVelocity(0.0);
				this.setAngularVelocity(Math.PI / 3);
				this.innerLoop();
				// this.update_VWCTRL_Parameter();
				// this.exec_VWCTRL_ALGO();
				// power+=1;
				// rightMotor.setPower(-((power<(u_r_max-15))?power:(u_r_max-15)));
				// leftMotor.setPower((power<(u_r_max-15))?power:(u_r_max-15));
			}
		}
	}

	private void stop() {
		this.setAngularVelocity(0.0);
		this.setVelocity(0.0);
		this.resetVW();
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.eold=0;
		this.eold_sp=0;
		//this.lock=20;
	}

	/**
	 * calculates the left and right angle speed of the both motors with given
	 * velocity and angle velocity of the robot
	 * 
	 * @param v
	 *            velocity of the robot in m/s
	 * @param omega
	 *            angle velocity of the robot in 1/s???, positiv für
	 *            mathematisch positive Rotation
	 * @return speed array containing the calculated angular velocity of left
	 *         (0) and right (1) wheel in degree/sec
	 */
	private double[] drive(double v, double omega) {
		v = v * 100;// Umrechnung in cm/s
		double v_l = 0; // Geschwindigkeit linkes Rad in cm/s
		double v_r = 0; // Geschwindigkeit rechtes Rad in cm/s
		double speedDegLeft = 0;
		double speedDegRight = 0;
		double speedDegMiddle = v / distPerDeg; // mittlere
												// Winkelgeschwindigkeit der
												// Räder in Grad pro sekunde
		if (omega != 0) {// Rotation vorhanden
			double r_m = v / omega; // Radius zum Rotationszentrum in cm
			if (r_m != 0) { // Rotation und Translation
				v_l = (r_m - r_robot) * v / r_m; // berechne Geschwindigkeit des
													// linken Rades in cm/s
				v_r = (r_m + r_robot) * v / r_m;

			} else { // nur Rotation
				v_l = -(omega * r_robot);// für math. positive umdrehung muss
											// linkes Rad rückwärts drehen
				v_r = omega * r_robot; // für math. positive umdrehung muss
										// rechtes Rad vorwärts drehen
			}
			speedDegLeft = v_l / distPerDeg; // Winkelgeschwindigkeit linkes Rad
												// in Grad/sec
			speedDegRight = v_r / distPerDeg; // Winkelgeschwindigkeit rechtes
												// Rad in Grad/sec
		} else { // nur Translation
			speedDegLeft = speedDegMiddle;
			speedDegRight = speedDegMiddle;
		}
		double[] speed = { speedDegLeft, speedDegRight };
		return speed;
	}

	/**
	 * Funktion zu Testzwecken, berechnet die Kennlinie w(PW)
	 */
	private void calcAngVelPerPercent() {
		this.update_VWCTRL_Parameter();
		leftMotor.forward();
		rightMotor.forward();
		if (zähler == 1) {
			monitor.writeControlComment("Akkuspannung: " + lejos.nxt.Battery.getVoltage());
		}
		/*
		 * if (first){ this.leftMotor.setPower(40);
		 * this.rightMotor.setPower(40); zähler++; }
		 */
		if (zähler < 100 /* && !first */) {
			this.leftMotor.setPower(power);
			this.rightMotor.setPower(power);
			this.encoderSumL += this.angleMeasurementLeft.getAngleSum();
			this.encoderSumR += this.angleMeasurementRight.getAngleSum();
			this.time += this.angleMeasurementLeft.getDeltaT();
			zähler++;
		}
		if (zähler == 100) {
			this.encoderSumL += this.angleMeasurementLeft.getAngleSum();
			this.encoderSumR += this.angleMeasurementRight.getAngleSum();
			this.time += this.angleMeasurementLeft.getDeltaT();
			double angVelL = (double) this.encoderSumL / (double) this.time;
			double angVelR = (double) this.encoderSumR / (double) this.time;
			monitor.writeControlComment("" + this.power + ": omega l: " + angVelL);
			monitor.writeControlComment("" + this.power + ": omega r: " + angVelR);
			this.encoderSumL = 0;
			this.encoderSumR = 0;
			this.time = 0;
			this.zähler = 0;
			/*
			 * if(first){ first=false; power=80; }
			 */
			this.power += 10;
		}
		if (power == 60) {
			curve = dest.stop;
			this.stop();
		}

	}

	/**
	 * Funktion zum Rücksetzen der Reglerparameter v/w-Control
	 */
	private void resetVW() {
		this.u_old_l = 0;
		this.u_old_r = 0;
		this.esuml_vw = 0;
		this.esumr_vw = 0;
		this.eoldl_vw = 0;
		this.newVW = true;
		this.eoldr_vw = 0;
	}

	private dest nearCurve() {
		double x = this.currentPosition.getX();
		double y = this.currentPosition.getY();
		if (this.backward){
			monitor.writeControlComment("backward");
			if((y>1.0 || y<0.2) && x>1.5 && x<2.0){
				if(y>0.85 || y<0.15){
					monitor.writeControlComment("Kurve 1 möglich back oben");
					return dest.right;
					
				}
				
				else return dest.no;
			}
			else if(x>3.0 || x<0.2){
				/*if(x>3.25){
					monitor.writeControlComment("Kurve 0 möglich");
					return dest.right;
					
				}*/
				return dest.no;
			}
			else return null;
		}
		
		else{
		if (y < 0.3 && x > 1.60) {
			// monitor.writeControlComment("nah Kurve1: x:"+x+" y:"+y);
			if(this.backward){
				if(y<0.2 && x >1.7){
					monitor.writeControlComment("Kurve 1 möglich: "+this.lock);
					return dest.right;// Kurve 1
				}
				else return dest.no;
			}
			else{
				if (y < 0.05 && x > 1.65) {
					monitor.writeControlComment("Kurve 1 möglich: "+this.lock);
					return dest.left;// Kurve 1
				} else
					return dest.no;
			}
		} else if (x > 1.65 && y > 0.3) {
			// monitor.writeControlComment("nah Kurve2: x:"+x+" y:"+y);
			if (y > 0.45 && x > 1.75) {
				monitor.writeControlComment("Kurve 2 möglich");
				return dest.left;// Kurve2
			} else
				return dest.no;
		} else if (x > 1.45 && x < 1.65 && y > 0.45) {
			// monitor.writeControlComment("nah Kurve3: x:"+x+" y:"+y);
			if (x > 1.45 && x < 1.60 && y > 0.55) {
				monitor.writeControlComment("Kurve 3 möglich");
				return dest.left;
			} else
				return dest.no;
		} else if (x > 1.2 && x < 1.55 && y < 0.45 && y > 0.25) {
			// monitor.writeControlComment("nah Kurve4: x:"+x+" y:"+y);
			if (x > 1.45 && x < 1.55 && y < 0.45 && y > 0.25) {
				monitor.writeControlComment("Kurve 4 möglich");
				return dest.right;// Kurve 4
			} else
				return dest.no;
		} else if (x < 0.5 && x > 0.25 && y < 0.45 && y > 0.25) {
			// monitor.writeControlComment("nah Kurve5: x:"+x+" y:"+y);
			if (x < 0.45 && x > 0.25 && y < 0.35 && y > 0.25) {
				monitor.writeControlComment("Kurve 5 möglich");
				return dest.right;// Kurve 5
			} else
				return dest.no;
		} else if (x < 0.35 && x > 0.15 && y > 0.45) {
			// monitor.writeControlComment("nah Kurve6: x:"+x+" y:"+y);
			if (x < 0.35 && x > 0.25 && y > 0.42) {
				monitor.writeControlComment("Kurve 6 möglich");
				Sound.beep();
				return dest.left;
			} else
				return dest.no;
		} else if (x < 0.15 && y > 0.3) {
			// monitor.writeControlComment("nah Kurve7: x:"+x+" y:"+y);
			if (x < 0.15 && y > 0.55) {
				monitor.writeControlComment("Kurve 7 möglich");
				return dest.left;// Kurve7
			} else
				return dest.no;
		} else if (x < 0.2 && y < 0.3) {
			// monitor.writeControlComment("nah Kurve8: x:"+x+" y:"+y);
			if(backward){
				if(x<0.2 && y<0.05){
					monitor.writeControlComment("Kurve 8 möglich back");
					return dest.right;
				}
				else return dest.no;
				
			}
			else{
				if (x < 0.05 && y < 0.16) {
					monitor.writeControlComment("Kurve 8 möglich");
					return dest.left;// Kurve8
				} else	return dest.no;
			}
		} else
			return null;
	}
	}

	private boolean nearSlot() {
		double d = Math.pow((this.currentPosition.getX() - this.startPosition.getX()), 2)
				+ Math.pow((this.currentPosition.getY() - this.startPosition.getY()), 2);
		if (this.firstPark && d < 0.04)
			return true;
		else
			return false;
	}
}
