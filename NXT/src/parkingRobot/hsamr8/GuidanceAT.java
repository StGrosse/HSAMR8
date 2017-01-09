package parkingRobot.hsamr8;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import lejos.geom.Line;
import lejos.nxt.LCD;

import parkingRobot.hsamr8.ControlRST;
import parkingRobot.hsamr8.HmiPLT;
import parkingRobot.hsamr8.NavigationAT;
import parkingRobot.hsamr8.PerceptionPMP;


/**
 * Main class for 'Hauptseminar AMR' project 'autonomous parking' for students of electrical engineering
 * with specialization 'automation, measurement and control'.
 * <p>
 * Task of the robotic project is to develop an mobile robot based on the Lego NXT system witch can perform
 * parking maneuvers on an predefined course. To fulfill the interdisciplinary aspect of this project the software
 * structure is divided in 5 parts: human machine interface, guidance, control, perception and navigation.
 * <p>
 * Guidance is to be realized in this main class. The course of actions is to be controlled by one or more finite
 * state machines (FSM). It may be advantageous to nest more than one FSM.
 * <p>
 * For the other parts there are interfaces defined and every part has to be realized in one main module class.
 * Every class (except guidance) has additionally to start its own thread for background computation.
 * <p>
 * It is important that data witch is accessed by more than one main module class thread is only handled in a
 * synchronized context to avoid inconsistent or corrupt data!
 */
public class GuidanceAT {
	
	/**
	 * states for the main finite state machine. This main states are requirements because they invoke different
	 * display modes in the human machine interface.
	 */
	public enum CurrentStatus {
		/**
		 * indicates that robot is following the line and maybe detecting parking slots
		 */
		DRIVING,
		/**
		 * indicates that robot is performing an parking maneuver
		 */
		INACTIVE,
		/**
		 * indicates that shutdown of main program has initiated
		 */
		EXIT
	}
	static float initHeading=0f;
	
	
	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatus currentStatus 	= CurrentStatus.INACTIVE;
	/**
	 * state in which the main finite state machine was running before entering the actual state
	 */
	protected static CurrentStatus lastStatus		= CurrentStatus.INACTIVE;
	
	
	/**
	 * one line of the map of the robot course. The course consists of a closed chain of straight lines.
	 * Thus every next line starts where the last line ends and the last line ends where the first line starts.
	 * This documentation for line0 hold for all lines.
	 */
	static Line line0 = new Line(  0,  0, 180,  0);
	static Line line1 = new Line(180,  0, 180, 60);
	static Line line2 = new Line(180, 60, 150, 60);
	static Line line3 = new Line(150, 60, 150, 30);
	static Line line4 = new Line(150, 30,  30, 30);
	static Line line5 = new Line( 30, 30,  30, 60);
	static Line line6 = new Line( 30, 60,   0, 60);
	static Line line7 = new Line(  0, 60,   0,  0);
	/**
	 * map of the robot course. The course consists of a closed chain of straight lines.
	 * Thus every next line starts where the last line ends and the last line ends where the first line starts.
	 * All above defined lines are bundled in this array and to form the course map.
	 */
	static Line[] map = {line0, line1, line2, line3, line4, line5, line6, line7};
	
	static int phase=1;
	
	
	/**
	 * main method of project 'ParkingRobot'
	 * 
	 * @param args standard string arguments for main method
	 * @throws Exception exception for thread management
	 */
	public static void main(String[] args) throws Exception {		
        currentStatus = CurrentStatus.INACTIVE;
        lastStatus    = CurrentStatus.EXIT;
		
		// Generate objects
		
		NXTMotor leftMotor  = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.C);
		
		IMonitor monitor = new Monitor();
		
		IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
		perception.calibrateLineSensors();
		
		INavigation navigation = new NavigationAT(perception, monitor);
		IControl    control    = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);
		INxtHmi  	hmi        = new HmiPLT(perception, navigation, control, monitor);
		
		monitor.startLogging();
				
		while(true) {
			showData(navigation, perception);
			
        	switch ( currentStatus )
        	{
				case DRIVING:
					// MONITOR (example)
//					monitor.writeGuidanceComment("Guidance_Driving");
					
					//Into action
					if ( lastStatus != CurrentStatus.DRIVING ){
						control.setCtrlMode(ControlMode.LINE_CTRL);
					}
					if(beispielsequenz1(navigation, control, monitor)){
						currentStatus=CurrentStatus.INACTIVE;
					}
					//While action				
					{
						//nothing to do here
					}					
					
					//State transition check
					lastStatus = currentStatus;
					if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
						currentStatus = CurrentStatus.INACTIVE;
					}else if ( Button.ENTER.isDown() ){
						currentStatus = CurrentStatus.INACTIVE;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT){
						currentStatus = CurrentStatus.EXIT;
					}
					
					//Leave action
					if ( currentStatus != CurrentStatus.DRIVING ){
						//nothing to do here
					}
					break;				
				case INACTIVE:
					//Into action
					if ( lastStatus != CurrentStatus.INACTIVE ){
						control.setCtrlMode(ControlMode.INACTIVE);
					}
					
					//While action
					{
						//nothing to do here
					}
					
					//State transition check
					lastStatus = currentStatus;
					if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
						currentStatus = CurrentStatus.DRIVING;						
					}else if ( Button.ENTER.isDown() ){
						currentStatus = CurrentStatus.DRIVING;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT){
						currentStatus = CurrentStatus.EXIT;
					}
					
					//Leave action
					if ( currentStatus != CurrentStatus.INACTIVE ){
						//nothing to do here
					}					
					break;
				case EXIT:
					hmi.disconnect();
					/** NOTE: RESERVED FOR FUTURE DEVELOPMENT (PLEASE DO NOT CHANGE)
					// monitor.sendOfflineLog();
					*/
					monitor.stopLogging();
					System.exit(0);
					break;
			default:
				break;
        	}
        		
        	Thread.sleep(100);        	
		}
	}
	
	
	/**
	 * returns the actual state of the main finite state machine as defined by the requirements
	 * 
	 * @return actual state of the main finite state machine
	 */
	public static CurrentStatus getCurrentStatus(){
		return GuidanceAT.currentStatus;
	}
	
	/**
	 * plots the actual pose on the robots display
	 * 
	 * @param navigation reference to the navigation class for getting pose information
	 */
	protected static void showData(INavigation navigation, IPerception perception){
		LCD.clear();	
		
		LCD.drawString("X (in cm): " + (navigation.getPose().getX()*100), 0, 0);
		LCD.drawString("Y (in cm): " + (navigation.getPose().getY()*100), 0, 1);
		LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading()/Math.PI*180), 0, 2);
		
		perception.showSensorData();
		
//    	if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
//			LCD.drawString("HMI Mode SCOUT", 0, 3);
//		}else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
//			LCD.drawString("HMI Mode PAUSE", 0, 3);
//		}else{
//			LCD.drawString("HMI Mode UNKNOWN", 0, 3);
//		}
	}
	private static boolean beispielsequenz1(INavigation navigation, IControl control, IMonitor monitor){
		if(phase==1){	
			navigation.setDetectionState(true);
			control.setVelocity(0.1);
			control.setAngularVelocity(0.0);
			control.setCtrlMode(ControlMode.VW_CTRL);
			
			if(navigation.getPose().getX()>1.49){
				phase=2;
				control.setCtrlMode(ControlMode.INACTIVE);
				navigation.setDetectionState(false);
				
			}   
			return false;
		}
		else if(phase==2){	
			
			control.setVelocity(0.0);
			control.setAngularVelocity(Math.PI/12);
			control.setCtrlMode(ControlMode.VW_CTRL);
			
			if(navigation.getPose().getHeading()>(Math.PI/2-Math.PI/240)){
				monitor.writeGuidanceComment("Winkel: "+navigation.getPose().getHeading());
				phase=3;
				control.setCtrlMode(ControlMode.INACTIVE);
				navigation.setDetectionState(false);			
			}
			return false;
		}
		else if(phase==3){
			
			control.setVelocity(0.05);
			control.setAngularVelocity(0.0);
			control.setCtrlMode(ControlMode.VW_CTRL);
			if(navigation.getPose().getY()>0.28){
				phase=4;
				control.setCtrlMode(ControlMode.INACTIVE);
				navigation.setDetectionState(false);
				initHeading=navigation.getPose().getHeading();
				
			}
			return false;
		}
		else if(phase==4){
		
			control.setVelocity(0.0);
			control.setAngularVelocity(Math.PI/6);
			control.setCtrlMode(ControlMode.VW_CTRL);
			if(navigation.getPose().getHeading()-initHeading>Math.PI/2-Math.PI/120){
				monitor.writeGuidanceComment("Winkel: "+navigation.getPose().getHeading());
				phase=5;
				control.setCtrlMode(ControlMode.INACTIVE);
				navigation.setLine(4);
				control.setStartTime(20); //Behelf
				navigation.setDetectionState(true);
				monitor.writeGuidanceComment("phase 5");

				
			}
			return false;
			
		}
		else if(phase==5){
			control.setCtrlMode(ControlMode.LINE_CTRL);
			Pose currentPose=navigation.getPose();
			if(currentPose.getX()>0.02 && currentPose.getY()<0.05){
				phase=6;
				control.setCtrlMode(ControlMode.INACTIVE);
				navigation.setDetectionState(false);
				
			}
			return false;
		}
		else if(phase==6){
			control.setDestination(1.5*Math.PI, 1.82, 0.62);		
			control.setCtrlMode(ControlMode.SETPOSE);
			if(control.getParkStatus()){
				phase=7;
				control.setCtrlMode(ControlMode.INACTIVE);
				navigation.setDetectionState(true);
				control.setStartTime(20);//Behelf
				navigation.setInv(true);
				
			}
			return false;
		}
		else if(phase==7){
			control.setCtrlMode(ControlMode.LINE_CTRL);
			Pose currentPose=navigation.getPose();
			if(currentPose.getX()<0.02){
				phase=8;
				control.setCtrlMode(ControlMode.INACTIVE);
				navigation.setDetectionState(false);
				
			}
			return false;
		}
		else if(phase==8){
			Pose pose = navigation.getPose();
			control.setDestination(0.0, pose.getX(),pose.getY() );
			control.setCtrlMode(ControlMode.SETPOSE);
			if(control.getParkStatus()){
				phase=9;
				control.setCtrlMode(ControlMode.INACTIVE);
				navigation.setDetectionState(false);
			}
			return false;

		}
		else if(phase==9){
			control.setDestination(0.0, 0.15, 0.02);
			control.setCtrlMode(ControlMode.SETPOSE);
			if(control.getParkStatus()){
				phase=10;
				control.setCtrlMode(ControlMode.INACTIVE);
				navigation.setDetectionState(false);
				float[]a={-0.1103703703703716f,1.8962962962963157f,-7.901234567901323f,7.023319615912314f};
				control.setPath(a, false, navigation.getPose(), new Pose(0.6f,-0.3f,0.0f));
			}
			return false;

		}
		else if(phase==10){
			
			control.setCtrlMode(ControlMode.PARK_CTRL);
			if(control.getParkStatus()){
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			return true;

		}
		
		else return false;

	}
	private static boolean beispielsequenz2(INavigation navigation, IControl control){
		if(phase==1){
			navigation.setDetectionState(false);
			control.setDestination(0.0, 0.15, 0.02);
			control.setCtrlMode(ControlMode.SETPOSE);
			if(control.getParkStatus()){
				phase=2;
				control.setCtrlMode(ControlMode.INACTIVE);
				float[]a={-0.1103703703703716f,1.8962962962963157f,-7.901234567901323f,7.023319615912314f};
				control.setPath(a, false, navigation.getPose(), new Pose(0.6f,-0.3f,0.0f));
			}
			return false;
		}
		else if(phase==2){
			control.setCtrlMode(ControlMode.PARK_CTRL);
			if(control.getParkStatus()){
				phase=3;
				control.setCtrlMode(ControlMode.INACTIVE);
				float[]a={-0.1103703703703716f,1.8962962962963157f,-7.901234567901323f,7.023319615912314f};
				control.setPath(a, true,new Pose(0.6f,-0.3f,0.0f), new Pose(0.15f,0.02f,0.0f));			
			}
			return false;
		}
		else if(phase==3){
			control.setCtrlMode(ControlMode.PARK_CTRL);
			if(control.getParkStatus()){
				phase=4;
				control.setCtrlMode(ControlMode.INACTIVE);
				navigation.setDetectionState(true);
				control.setStartTime(20);//Behelf
			}
			return false;
		}
		else if(phase==4){
			control.setCtrlMode(ControlMode.LINE_CTRL);
			Pose pose=navigation.getPose();
			if(pose.getX()>1.77 && pose.getY()>0.02 &&pose.getHeading()>Math.PI*(1-1/40)){
				phase=5;
				control.setCtrlMode(ControlMode.INACTIVE);
				navigation.setDetectionState(false);			
				float[] a={1.7821411951148907f,-0.21733935376707336f,5.674972015028288f,-8.049605695065111f};
				control.setPath(a, false, new Pose(0.02f,1.78f,0.0f), new Pose(0.45f,2.1f,0.0f));
			}
			return false;
		}
		else if(phase==5){
			control.setCtrlMode(ControlMode.PARK_CTRL);
			if(control.getParkStatus()){
				phase=6;
				control.setCtrlMode(ControlMode.INACTIVE);
				float[] a={1.7821411951148907f,-0.21733935376707336f,5.674972015028288f,-8.049605695065111f};
				control.setPath(a, true, new Pose(0.45f,2.1f,0.0f),new Pose(0.02f,1.78f,0.0f));
			}
			return false;
		}
		else if(phase==6){
			control.setCtrlMode(ControlMode.PARK_CTRL);
			if(control.getParkStatus()){
				phase=7;
				control.setCtrlMode(ControlMode.INACTIVE);
				navigation.setDetectionState(true);
			}
			else return false;
		}
		else if(phase==7){
			control.setCtrlMode(ControlMode.LINE_CTRL);
			return false;
		}
		return false;
	}
}