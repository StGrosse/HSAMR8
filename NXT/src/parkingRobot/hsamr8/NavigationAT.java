package parkingRobot.hsamr0;

import java.util.ArrayList;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.nxt.Sound;
import lejos.robotics.navigation.Pose;


import parkingRobot.INavigation;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;


import parkingRobot.hsamr0.NavigationThread;
import parkingRobot.hsamr0.GuidanceAT;


/**
 * A executable basic example implementation of the corresponding interface provided by the Institute of Automation with
 * limited functionality:
 * <p>
 * In this example only the both encoder sensors from the {@link IPerception} implementation are used for periodically calculating
 * the robots position and corresponding heading angle (together called 'pose'). Neither any use of the map information or other
 * perception sensor information is used nor any parking slot detection is performed, although the necessary members are already
 * prepared. Advanced navigation calculation and parking slot detection should be developed and invented by the students.  
 * 
 * @author IfA
 */
public class NavigationAT implements INavigation{
	
	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
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
	 * reference to {@link IPerception.OdoSensor} class for mouse odometry sensor to measure the ground displacement
	 * between actual an last request
	 */
	IPerception.OdoSensor mouseodo = null;	
		
	/**
	 * reference to data class for measurement of the mouse odometry sensor to measure the ground displacement between
	 * actual an last request and the corresponding time difference
	 */
	IPerception.OdoDifferenceMeasurement mouseOdoMeasurement = null;
	
	/**
	 * distance from optical sensor pointing in driving direction to obstacle in mm
	 */
	double frontSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the front)
	 */
	double frontSideSensorDistance	=	0;
	/**
	 * distance from optical sensor pointing in opposite of driving direction to obstacle in mm
	 */
	double backSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the back)
	 */
	double backSideSensorDistance	=	0;


	/**
	 * robot specific constant: radius of left wheel
	 */
	static final double LEFT_WHEEL_RADIUS	= 	0.028; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final double RIGHT_WHEEL_RADIUS	= 	0.028; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE		= 	0.119; // only rough guess, to be measured exactly and maybe refined by experiments

	
	/**
	 * map array of line references, whose corresponding lines form a closed chain and represent the map of the robot course
	 */
	Line[] map 								= null;
	/**
	 * reference to the corresponding main module Perception class
	 */
	IPerception perception 	        		= null;
	/**
	 * reference to the corresponding main module Monitor class
	 */
	IMonitor monitor = null;
	/**
	 * indicates if parking slot detection should be switched on (true) or off (false)
	 */
	boolean parkingSlotDetectionIsOn		= false;
	/**
	 * pose class containing bundled current X and Y location and corresponding heading angle phi
	 */
	Pose pose								= new Pose();
    
	//Pose lastPose[]=new Pose[5];
	/**
	 * pose class containing bundled unprocessed X and Y location and corresponding heading angle phi
	 */
	//Pose unprocessedPose = new Pose();
	/**
	 * thread started by the 'Navigation' class for background calculating
	 */
	NavigationThread navThread = new NavigationThread(this);

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
	
	 static Line linestart = new Line(0,0,0,0);
	/**
	 * map of the robot course. The course consists of a closed chain of straight lines.
	 * Thus every next line starts where the last line ends and the last line ends where the first line starts.
	 * All above defined lines are bundled in this array and to form the course map.
	 */
	//static Line[] map1 = {line0, line1, line2, line3, line4, line5, line6, line7};
	
	
	
	//Line ReferenceLine;
	
	//float currentAngle;
	int currentLine;
	//boolean detectionLine=false;
	//boolean online=false;
	//boolean parkingStart=false;
	//boolean detectcorner=false;
	
	

	/*
	 * refines the robot pose from the Mouse Sensor measurment
	 * 
	 */
	//float deltaT_Mau=((float)this.mouseOdoMeasurement.getDeltaT())/1000;
	//double x_change=this.mouseOdoMeasurement.getUSum();
	//double y_change=this.mouseOdoMeasurement.getVSum();
	/*
	 * parkinSlotsuchen parameter
	 */
	INavigation.ParkingSlot ParkingSlot = null;
	INavigation.ParkingSlot Slots[] = null;
	int parken_ID; //parkenSlotnumber
	ArrayList<ParkingSlot> slotList = new ArrayList<ParkingSlot>();
	boolean detectionStatus=false;
	boolean parkingSuchen=false;
	Point backBoundaryPosition = null;
	Point frontBoundaryPosition = null;
	int measurementQuality;

	boolean parkenSlotSuch0 = false;
	boolean parkenSlotSuch1 = false;
	boolean parkenSlotSuch4 = false;
	
	boolean signal=false;
	
	double parkenGutLength = 0.02;
	double parkenGutLength1 = 0.03;//distance from centerl to abstandsensor
	double parkenGutLength2 = 0.15;
	
	double frontparkPosition = 0;
	double backparkPosition = 0;
	double parkingSlotLength=0;
	/*
	 * the parameter 
	 */
	//static final int maxDis=10;
	private static final double SameSlotDifference = 20;
	//float lineWinkel=0;
	/**
	 * returns the map
	 */
	public Line[] getMap() {
		return map;
	}
	
	// ParkingSlot[] parkingSlotList=new ParkingSlot[20]; 
	
	
	
	/**
	 * provides the reference transfer so that the class knows its corresponding perception object (to obtain the measurement
	 * information from) and starts the navigation thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param monitor corresponding main module Monitor class object
	 */
	public NavigationAT(IPerception perception, IMonitor monitor){
		this.perception   = perception;
		this.monitor = monitor;
		this.encoderLeft  = perception.getNavigationLeftEncoder();
		this.encoderRight = perception.getNavigationRightEncoder();
		this.mouseodo = perception.getNavigationOdo();		
		
		
        // Definition der S[alten der Tabelle der Monitor-Schnittstelle
		monitor.addNavigationVar("X_wert");
		monitor.addNavigationVar("Y_wert");
		monitor.addNavigationVar("Alpha_wert");
		//monitor.addNavigationVar("Slots");
		
		
		navThread.setPriority(Thread.MAX_PRIORITY - 1);
		navThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		navThread.start();
		
	}
	
	
	// Inputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setMap(lejos.geom.Line[])
	 */
	public void setMap(Line[] map){
		this.map = map;
	}
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setDetectionState(boolean)
	 */
	public void setDetectionState(boolean isOn){
		this.parkingSlotDetectionIsOn = isOn;
	}
	
	
	// 	Class control
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#updateNavigation()
	 */
	public synchronized void updateNavigation(){	
		this.updateSensors();
		this.calculateLocationLocation();
		// Detection->close
		//if (this.parkingSlotDetectionIsOn)
				this.detectParkingSlot();
		
		
		// MONITOR (example)		
		//monitor.writeNavigationComment("Navigation");
		monitor.writeNavigationVar("X_wert", "" +  this.getPose().getX());
		monitor.writeNavigationVar("Y_wert", "" +  this.getPose().getY());
		monitor.writeNavigationVar("Alpha_wert", "" +  this.getPose().getHeading());

	}
	
	
	// Outputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#getPose()
	 */
	public synchronized Pose getPose(){
		return this.pose;
	}
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#getParkingSlots()
	 */
	public synchronized ParkingSlot[] getParkingSlots() {
		return Slots;
	}
	
	
	// Private methods
	
	/*
	 * calls the perception methods for obtaining actual measurement data and writes the data into members
	 */
	private void updateSensors(){		
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
		this.angleMeasurementLeft  	= this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight 	= this.encoderRight.getEncoderMeasurement();

		this.mouseOdoMeasurement	= this.mouseodo.getOdoMeasurement();

		this.frontSensorDistance	= perception.getFrontSensorDistance();
		this.frontSideSensorDistance = perception.getFrontSideSensorDistance();
		this.backSensorDistance		= perception.getBackSensorDistance();
		this.backSideSensorDistance	= perception.getBackSideSensorDistance();
	}		 	
	
	/**
	 * calculates the robot pose from the encoder measurements and linesensor
	 */
	private void calculateLocationLocation(){
		// calculate the robot pose with encoder
		double leftAngleSpeed 	= this.angleMeasurementLeft.getAngleSum()  / ((double)this.angleMeasurementLeft.getDeltaT()/1000);  //degree/seconds
		double rightAngleSpeed 	= this.angleMeasurementRight.getAngleSum() / ((double)this.angleMeasurementRight.getDeltaT()/1000); //degree/seconds

		double vLeft		= (leftAngleSpeed  * Math.PI * LEFT_WHEEL_RADIUS ) / 180 ; //velocity of left  wheel in m/s
		double vRight		= (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180 ; //velocity of right wheel in m/s		
		double w 			= (vRight - vLeft) / WHEEL_DISTANCE; //angular velocity of robot in rad/s
		
		Double R 			= new Double(( WHEEL_DISTANCE / 2 ) * ( (vLeft + vRight) / (vRight - vLeft) ));								
		
		double ICCx 		= 0;
		double ICCy 		= 0;

		double xResult 		= 0;
		double yResult 		= 0;
		double angleResult 	= 0;
		
		double deltaT       = ((double)this.angleMeasurementLeft.getDeltaT())/1000;
		
	    //detectline();         //make sure the robot on line
		//ReferenceLine=whichLine(pose,maxDis);//know which line the robot is on
		//detectCorner();       // Corner or not 
		
		if (R.isNaN()) { //robot don't move
			xResult			= this.pose.getX();
			yResult			= this.pose.getY();
			angleResult 	= this.pose.getHeading();
		} else if (R.isInfinite()) { //robot moves straight forward/backward, vLeft==vRight
			xResult			= this.pose.getX() + vLeft * Math.cos(this.pose.getHeading()) * deltaT;
			yResult			= this.pose.getY() + vLeft * Math.sin(this.pose.getHeading()) * deltaT;
			angleResult 	= this.pose.getHeading();
		} else {			
			ICCx = this.pose.getX() - R.doubleValue() * Math.sin(this.pose.getHeading());
			ICCy = this.pose.getY() + R.doubleValue() * Math.cos(this.pose.getHeading());
		
			xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
			yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
			angleResult 	= this.pose.getHeading() + w * deltaT;
			if (angleResult >Math.PI * 1.77) {
				angleResult = 0;
				xResult = 0.02;
				yResult = 0.0;
				currentLine=0;
			}
		}
		//line0
		if((this.pose.getHeading() < Math.PI * 0.10)&& (this.pose.getHeading() >- Math.PI * 0.10)){
		         currentLine=0;
				
				if (parkingSlotDetectionIsOn) {
					currentLine=0;
					if ((this.pose.getX() >0.1)&& (this.pose.getX()<1.65))
					{
						if ((this.pose.getHeading() > Math.PI * 0.01)|| (this.pose.getHeading() < -Math.PI * 0.01)) {
							angleResult = 0;
						}
						if (this.pose.getY() < -0.01) {
							yResult = 0;
						}
						if (this.pose.getY() > 0.01) {
							yResult = 0;
						}
						
				   }
					if(this.pose.getX()>1.83){
						xResult=1.78;
					}
							currentLine=0;

					}
					
						currentLine=0;
				}
				
			
			
		if(currentLine==0){
			monitor.writeNavigationComment("enter loopline0");
			if ((this.pose.getX() >0.03)&& (this.pose.getX()<1.65)) {
				if ((this.pose.getHeading() > Math.PI * 0.01)|| (this.pose.getHeading() < -Math.PI * 0.01))
				{
					angleResult = 0;
				}
				if (this.pose.getY() < -0.01) {
					yResult = 0;
				}
				if (this.pose.getY() > 0.01) {
					yResult = 0;
				}
		      }     
			if(this.pose.getX()>=2.0){
				xResult=1.78;
			}
			
			currentLine=0;
		}
		
		
		//Line=1
		if ((this.pose.getHeading() >Math.PI * 0.44) && (this.pose.getHeading() < Math.PI * 0.55)&& (currentLine == 0)) {
			monitor.writeNavigationComment("enter line1");
			xResult=1.78;
			angleResult=0.5*Math.PI;
            currentLine=1;
			if (parkingSlotDetectionIsOn) {
				xResult=1.78;
				currentLine=1;
				if ((this.pose.getY() > 0.05) && (this.pose.getY() < 0.65)) {
					if ((this.pose.getHeading() < Math.PI * 0.49) || (this.pose.getHeading() > Math.PI * 0.51)) {
						//angleResult = Math.PI * 0.5;
						angleResult=0.5*Math.PI;
					}
					if (this.pose.getX() > 1.79) {
				    xResult = 1.78;
			      }
			       if (this.pose.getX() < 1.77) {
				   xResult = 1.78;
		         	}
				}
			
			
			}
			
		}
		
		if (currentLine== 1) {
           //xResult=1.78;
            monitor.writeNavigationComment("enterloop1");
			currentLine=1;
		//	if (parkingSlotDetectionIsOn) {
			//	xResult=1.78;
				if ((this.pose.getY() > 0.05) && (this.pose.getY() < 0.55)) {
					if ((this.pose.getHeading() < Math.PI * 0.49) || (this.pose.getHeading() >Math.PI * 0.51)) {
						angleResult =0.5*Math.PI;
					}
					if (this.pose.getX() >1.79) {
						xResult = 1.78;
					}
					if (this.pose.getX() <1.77) {
						xResult = 1.78;
						 monitor.writeNavigationComment("enterloop1.1.78");

					}
					
				}
				currentLine = 1;
		//	}

			

		}
		/*
		 * line 2
		 */

		if ((this.pose.getHeading() > Math.PI * 0.85) && (this.pose.getHeading() < Math.PI * 1.136)&& (currentLine== 1)) {
			monitor.writeNavigationComment("enter line2");
			yResult = 0.60;
			angleResult = Math.PI;
			currentLine = 2;
			
		/*	if (parkingSlotDetectionIsOn) {
				yResult=0.6;
				if ((this.pose.getX() < 1.85) && (this.pose.getX() >1.40)) {
					if ((this.pose.getHeading()<Math.PI * 0.99) || (this.pose.getHeading() >Math.PI * 1.01)) {
						angleResult = Math.PI;
					}
					if (this.pose.getY() < 0.59) {
						yResult = 0.6;
					}
					if (this.pose.getY() > 0.61) {
						yResult = 0.6;
					}

			}
			}*/
		}
		if (currentLine == 2) {
              currentLine=2;
              monitor.writeNavigationComment("enter loopline2");

		   //if (parkingSlotDetectionIsOn) {

				    monitor.writeNavigationComment("enter loopline2¡ª¡ªtest");
			      if ((this.pose.getX() < 1.78) && (this.pose.getX() >1.55)) {
					if ((this.pose.getHeading()<Math.PI * 0.99) || (this.pose.getHeading() >Math.PI * 1.01)) {
						angleResult = Math.PI;
					}
			//		if (this.pose.getY() < 0.59)
					if (this.pose.getY() < 0.59)
					{
						yResult = 0.6;
					}
					if (this.pose.getY() > 0.62) {
						yResult = 0.6;
					}
					
			//}
				currentLine = 2;
			}
	
			currentLine=2;

           }
		/*
		 * line 3
		 */

		if ((this.pose.getHeading() > Math.PI * 1.40) && (this.pose.getHeading() < Math.PI * 1.67)&& (currentLine == 2)) {
			monitor.writeNavigationComment("enter line3");
			xResult = 1.50;
			angleResult = Math.PI * 1.5;
			currentLine = 3;
		//	if (parkingSlotDetectionIsOn) {
				if ((this.pose.getY() > 0.35) && (this.pose.getY() < 0.65)) {
				  angleResult = Math.PI * 1.5;
				  currentLine=3;
					if ((this.pose.getHeading() < Math.PI * 1.49) || (this.pose.getHeading() >Math.PI * 1.51)) {
						angleResult = Math.PI * 1.5;
					}
					if (this.pose.getX() > 1.51) {
						xResult = 1.50;
					}
					if (this.pose.getX() < 1.49) {
						xResult = 1.50;
					}
					
		//		}
				if(this.pose.getY()<0.3){
					yResult=0.3;
				}
			
				
				}
			}
		if (currentLine == 3) {
			monitor.writeNavigationComment("enter loopline3");
			currentLine=3;
	      //    if (parkingSlotDetectionIsOn) {
				    currentLine=3;
				   // xResult=0.5;
				if ((this.pose.getY() > 0.35) && (this.pose.getY() < 0.65)) {
					if ((this.pose.getHeading() < Math.PI * 1.49) || (this.pose.getHeading() >Math.PI * 1.51)) {
						angleResult = Math.PI * 1.5;
					}
					if (this.pose.getX() > 1.51) {
						xResult = 1.50;
					}
					if (this.pose.getX() < 1.49) {
						xResult = 1.50;
					}
					
				
				}
				if(this.pose.getY()<0.3){
					yResult=0.3;
				}
		//}
			
		
		}
		/*
		 * line 4
		 */
		if ((this.pose.getHeading() > Math.PI * 0.8) && (this.pose.getHeading() < Math.PI * 1.25)&& (currentLine == 3)) {
			monitor.writeNavigationComment("enter line4");
			  currentLine=4;
			  yResult=0.30;
			  angleResult=Math.PI;
			  
			if (parkingSlotDetectionIsOn) {
				//yResult=0.3;
				//angleResult=Math.PI;
				currentLine=4;
				if ((this.pose.getX() <1.65) && (this.pose.getX() >0.45)) {
			
					if ((this.pose.getHeading() < Math.PI * 0.99) || (this.pose.getHeading() > Math.PI * 1.01)) {
						angleResult = Math.PI;
					}
				if (this.pose.getY() < 0.29) {
					yResult = 0.30;
				}
				if (this.pose.getY() > 0.31) {
					yResult = 0.30;

				}
				
				
			  }
				
				currentLine = 4;
			}
	
			
		}
		//Linien 4 noch Problem(Y(cm))
		if (currentLine == 4) {
			monitor.writeNavigationComment("enter loopline4");
              currentLine=4;
		   	//if (parkingSlotDetectionIsOn) {
				//yResult=0.3;
				if ((this.pose.getX() <1.65) && (this.pose.getX() >0.45)) {
					//yResult=0.3;//test
					if ((this.pose.getHeading() < Math.PI * 0.99)|| (this.pose.getHeading()> Math.PI * 1.01)) {
						    angleResult = Math.PI;
					}   
					if (this.pose.getY() < 0.29) {
							yResult = 0.30;
					}
					if (this.pose.getY() > 0.31) {
						    yResult = 0.30;
				     }
				}
				

               currentLine = 4;
			//}

		}
		/*
		 * line 5
		 */

		if ((this.pose.getHeading()> Math.PI * 0.40) && (this.pose.getHeading() <Math.PI * 0.60)&& (currentLine== 4)) {
			monitor.writeNavigationComment("enter line5");
            xResult=0.3;
			currentLine = 5;
			angleResult=0.5*Math.PI;
			if (parkingSlotDetectionIsOn) {
              
				//if ((this.pose.getY() > 0.35) && (this.pose.getY() < 0.60)) {
					if ((this.pose.getHeading() > Math.PI * 0.51) || (this.pose.getHeading()< Math.PI * 0.49)){
						angleResult = Math.PI*0.5;
					}
				
					if (this.pose.getX() > 0.31) {
						xResult = 0.3;
					}
					if (this.pose.getX() < 0.29) {
						xResult = 0.3;
					}
				}
				currentLine = 5;
			//}
		}
		if (currentLine == 5) {
			monitor.writeNavigationComment("enter loopline5");
			//if (parkingSlotDetectionIsOn) {
                 currentLine=5;
				if ((this.pose.getY() > 0.35) && (this.pose.getY() < 0.55)) {
					if ((this.pose.getHeading() > Math.PI * 0.51) || (this.pose.getHeading()< Math.PI * 0.49)){
						angleResult = Math.PI*0.5;
					}
				
					if (this.pose.getX() > 0.31) {
						xResult = 0.3;
					}
					if (this.pose.getX() < 0.29) {
						xResult = 0.3;
					}
				//}
				currentLine = 5;
			}

		
		}
		/*
		 * line 6
		 */
		if ((this.pose.getHeading() > Math.PI * 0.90) && (this.pose.getHeading() <Math.PI * 1.10)&& (currentLine== 5)) {
			monitor.writeNavigationComment("enter line6");
			yResult=0.6;
			angleResult=Math.PI;
			currentLine = 6;
			if (parkingSlotDetectionIsOn) {
				//if ((this.pose.getX() <0.25) && (this.pose.getX() >0.10)) {
					if ((this.pose.getHeading() > Math.PI *1.10) || (this.pose.getHeading()< Math.PI * 0.90)){
						angleResult = Math.PI;
					}
					if (this.pose.getY() < 0.59) {
						yResult = 0.6;
					}

					if (this.pose.getY() > 0.61) {
						yResult = 0.6;
					}
				//}
			
                 currentLine = 6;
			}
		}
		if (currentLine == 6) {
			monitor.writeNavigationComment("enter loopline6");
			  currentLine = 6;
			 // yResult=0.6;
			//if (parkingSlotDetectionIsOn) {
				if ((this.pose.getX() <0.3) && (this.pose.getX() >0.08)) {
					if ((this.pose.getHeading() > Math.PI *1.10) || (this.pose.getHeading()< Math.PI * 0.90)){
						angleResult = Math.PI;
					}
					if (this.pose.getY() < 0.59) {
						yResult = 0.6;
					}

					if (this.pose.getY() > 0.61) {
						yResult = 0.6;
					}
				}
			
                 currentLine = 6;
			//}

		
		}
		/*
		 * line 7
		 */

		if ((this.pose.getHeading() >Math.PI * 1.40) && (this.pose.getHeading() <Math.PI * 1.60)&& (currentLine == 6)) {
			monitor.writeNavigationComment("enter line7");
             currentLine=7;
             angleResult=1.5*Math.PI;
             xResult=0;
			
			if (parkingSlotDetectionIsOn) {
				currentLine=7;
				xResult=0;
				angleResult=1.5*Math.PI;
				//if ((this.pose.getY() > 0.10) && (this.pose.getY() < 0.58)) {
					if ((this.pose.getHeading() < Math.PI * 1.49)|| (this.pose.getHeading() > Math.PI * 1.51)) {
						angleResult = Math.PI * 1.5;
					}
				
				if (this.pose.getX() < -0.01) {
					xResult = 0;
				}

				if (this.pose.getX() > 0.01) {
					xResult = 0;
				}
				currentLine = 7;
			}
			//}
			
		}
		//if ((this.pose.getHeading() >Math.PI * 1.40) && (this.pose.getHeading() <Math.PI * 1.67) && (currentLine== 7)) {
		if (currentLine== 7){
			monitor.writeNavigationComment("enter loopline7");
		//	if (parkingSlotDetectionIsOn) {
			if ((this.pose.getHeading() < Math.PI * 1.49) || (this.pose.getHeading() > Math.PI * 1.51)) {
				angleResult = Math.PI * 1.5;
				}
				
				if (this.pose.getX() < -0.01) {
					xResult = 0;
				}

				if (this.pose.getX() > 0.01) {
					xResult = 0;
				}
			//}

			currentLine = 7;
			//}
		}
		
		this.pose.setLocation((float) xResult, (float) yResult);
		this.pose.setHeading((float) angleResult);
		
	}


/*
 * Returns the square of the distance from a point to a line segment.
 *  The distance measured is the distance between the specified point and 
 *  the closest point between the specified end points. 
 *  If the specified point intersects the line segment in between the end points, this method returns 0.0.
 */
	@SuppressWarnings("unused")
	private static float ptSegDistSq(Line l, float px, float py){
		
		float x1= (float)l.getX1();
		float x2= (float)l.getX2();
		float y1= (float)l.getY1();
		float y2 =(float)l.getY2();
		x2-=x1;
		y2-=y1;
		px-=x1;
		py-=y1;
	    float dist;
		if (px * x2 + py * y2 <= 0.0) { // P*A
		        dist = px * px + py * py;
		    } else {
		        px = x2 - px; // P = A - P = (x2 - px, y2 - py)
		        py = y2 - py;
		        if (px * x2 + py * y2 <= 0.0) { // P*A
		            dist = px * px + py * py;
		        } else {
		            dist = px * y2 - py * x2;
		            dist = dist * dist / (x2 * x2 + y2 * y2); // pxA/|A|
		        }
		    }
		    if (dist < 0) {
		        dist = 0;
		    }
		    return dist;
	}
	
	




	
	
	 
	/**
	 * 	detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot(){
		
		switch (currentLine) {
			case 0:
				if (this.pose.getX() <1.70) {
					
					monitor.writeNavigationComment("parkenLine0");
				   if ((parkenSlotSuch0 == false) && (this.frontSideSensorDistance >280)) {
						//if (this.frontSideSensorDistance >280){
						backBoundaryPosition = new Point((float) (this.pose.getX() + parkenGutLength1), this.pose.getY());
						monitor.writeNavigationComment("back");
						//backBoundaryPosition = new Point(this.pose.getX()+parkenGutLength1, this.pose.getY());
						//parkenSlotSuch0 = true;
					//	if(this.pose.getHeading()<0.1*Math.PI&&this.pose.getHeading()>-0.1*Math.PI){
						 measurementQuality=5;
							parkenSlotSuch0 = true;
							
						//}
					//	else{
						//	measurementQuality=1;
							//parkenSlotSuch0 = false;
					//	}

					}
					if ((parkenSlotSuch0 == true) && (this.frontSideSensorDistance <170)) {
						//frontBoundaryPosition = new Point((float) (this.pose.getX() - parkenGutLength1),
						frontBoundaryPosition =new Point((float) (this.pose.getX() + parkenGutLength1), this.pose.getY());
						parkenSlotSuch0 = false;
						signal = true;
                        monitor.writeNavigationComment("front");
					}
				}
				break;
			case 1:
				if ((this.pose.getY() > 0.05) && (this.pose.getY() < 0.56)) {
					monitor.writeNavigationComment("parkenLine1");
					if ((parkenSlotSuch0 == false) && (this.frontSideSensorDistance > 280)) {
						backBoundaryPosition = new Point(this.pose.getX(), (float) (this.pose.getY() +parkenGutLength1));
					    //parkenSlotSuch1= true;
					//	if(this.pose.getHeading()<0.44*Math.PI&&this.pose.getHeading()>0.55*Math.PI){
							measurementQuality=5;//gut measurment
							parkenSlotSuch1 = true;
						//}
					//	else{
						//	measurementQuality=1;
					//		parkenSlotSuch1 = false;
					//}
				}
					if ((parkenSlotSuch1 == true) && (this.frontSideSensorDistance < 200)) {
						frontBoundaryPosition = new Point(this.pose.getX(),(float) (this.pose.getY() +parkenGutLength1));
						//frontBoundaryPosition = new Point(this.pose.getX(),this.pose.getY());
						parkenSlotSuch1 = false;
						signal = true;
					}
				}
				break;
			
			case 4:
				if ((this.pose.getX() <1.27) && (this.pose.getX() >0.53)) {
					monitor.writeNavigationComment("parkenLine4");
					if ((parkenSlotSuch4 == false) && (this.frontSideSensorDistance > 280)) {
						
						backBoundaryPosition = new Point((float) (this.pose.getX() -parkenGutLength1), this.pose.getY());
						//backBoundaryPosition = new Point(this.pose.getX() , this.pose.getY());
						//parkenSlotSuch4 = true;
				//		if(this.pose.getHeading()<1.05*Math.PI&&this.pose.getHeading()>-1.05*Math.PI){
							measurementQuality=5;//gut measurment
							parkenSlotSuch0 = true;
				//		}
			//			else{
				//			measurementQuality=1; //bad measurment
					//		parkenSlotSuch0 = false;
				//	}
				}
					if ((parkenSlotSuch4 == true) && (this.frontSideSensorDistance < 180)) {
					
						frontBoundaryPosition =new Point((float) (this.pose.getX() -parkenGutLength1), this.pose.getY());
						parkenSlotSuch4= false;
						signal = true;
					}
				}
				break;
			}

			if (signal == true) {
				if ((this.frontBoundaryPosition != null) && (this.backBoundaryPosition != null)) {
					frontparkPosition = Math.abs(this.frontBoundaryPosition.getX()) + this.frontBoundaryPosition.getY();
					backparkPosition = Math.abs(this.backBoundaryPosition.getX()) + this.frontBoundaryPosition.getY();
					//frontparkPosition = Math.abs(this.frontBoundaryPosition.getX());
					//backparkPosition = Math.abs(this.backBoundaryPosition.getX());
					parkingSlotLength = frontparkPosition - backparkPosition;
					parken_ID++;
					monitor.writeNavigationComment("line1");
				}

				if (parkingSlotLength>= 0.45) {
					ParkingSlotStatus status = ParkingSlotStatus.GOOD;
					ParkingSlot getParkingSlots = new ParkingSlot(parken_ID, backBoundaryPosition,
							frontBoundaryPosition,ParkingSlotStatus.GOOD,measurementQuality);
					
					slotList.add(getParkingSlots);
					this.monitor.writeNavigationComment("X_BACK "+getParkingSlots.getBackBoundaryPosition().x);
					this.monitor.writeNavigationComment("Y_BACK "+getParkingSlots.getBackBoundaryPosition().y);
				    this.monitor.writeNavigationComment("X_FRONT "+getParkingSlots.getFrontBoundaryPosition().x);
				    this.monitor.writeNavigationComment("Y_FRONT "+getParkingSlots.getFrontBoundaryPosition().y);
				    this.monitor.writeNavigationComment("Parken_ID "+getParkingSlots.getID());
				    this.monitor.writeNavigationComment("STATUS "+getParkingSlots.getStatus());
				    this.monitor.writeNavigationComment( "grose des Arrays:"+slotList.size());
					Slots = new ParkingSlot[slotList.size()];
					Slots = slotList.toArray(Slots);  //create a new array
					
					/*this.monitor.writeNavigationComment("X_BACK "+Slots[0].getBackBoundaryPosition().x);
					this.monitor.writeNavigationComment("Y_BACK "+Slots[0].getBackBoundaryPosition().y);
				    this.monitor.writeNavigationComment("X_FRONT "+Slots[0].getFrontBoundaryPosition().x);
				    this.monitor.writeNavigationComment("Y_FRONT "+Slots[0].getFrontBoundaryPosition().y);
				    this.monitor.writeNavigationComment("Parken_ID "+Slots[0].getID());
				    this.monitor.writeNavigationComment("STATUS "+Slots[0].getStatus());
				    this.monitor.writeNavigationComment( "TEST:  grose des Arrays:"+Slots.length);*/
					
					//if the two Slots are same->remove 
				//    for(int i=0;i<Slots.length;i++){
					 	//if(sameSlot(Slots[i],Slots[Slots.length-1])){
						//	slotList.remove(i);
							//i--;
				    	 

				} else if ((parkingSlotLength > 0) && (parkingSlotLength < 0.45)) {
					ParkingSlotStatus status = ParkingSlotStatus.BAD;
					ParkingSlot getParkingSlots = new ParkingSlot(parken_ID, backBoundaryPosition,
							frontBoundaryPosition,status, measurementQuality);
				
					slotList.add(getParkingSlots);
					this.monitor.writeNavigationComment("X_BACK "+getParkingSlots.getBackBoundaryPosition().x);
					this.monitor.writeNavigationComment("Y_BACK "+getParkingSlots.getBackBoundaryPosition().y);
				    this.monitor.writeNavigationComment("X_FRONT "+getParkingSlots.getFrontBoundaryPosition().x);
				    this.monitor.writeNavigationComment("Y_FRONT "+getParkingSlots.getFrontBoundaryPosition().y);
				    this.monitor.writeNavigationComment("Parken_ID "+getParkingSlots.getID());
				    this.monitor.writeNavigationComment("STATUS "+getParkingSlots.getStatus());
					Slots = new ParkingSlot[slotList.size()];
					 this.monitor.writeNavigationComment( "grose des Arrays:"+slotList.size());
					Slots = slotList.toArray(Slots);
				//	for(int i=0;i<Slots.length;i++){
					//	if(sameSlot(Slots[i],Slots[Slots.length-1])){
						//	slotList.remove(i);
							//i--;
					//}
						}
						
				
				
				 else {
					ParkingSlotStatus status = ParkingSlotStatus.RESCAN;
					ParkingSlot getParkingSlots = new ParkingSlot(parken_ID, backBoundaryPosition,
							frontBoundaryPosition,status, measurementQuality);
					parken_ID--;
				}
				signal = false;
				this.backBoundaryPosition = null;
				this.frontBoundaryPosition = null;
				

			}
			
			
			return; // has to be implemented by students
			
			
		
 }
	
			/*
			 * whether two parkenSlots are same or not
			 * @return true: the two slots are same ,false: the two slots are false
			 */
			public boolean sameSlot(ParkingSlot x, ParkingSlot y){
				if((Math.abs((x.getBackBoundaryPosition().getX())-
						y.getBackBoundaryPosition().getY())<SameSlotDifference)
						||((Math.abs(x.getFrontBoundaryPosition().getX())-
								y.getFrontBoundaryPosition().getY())<SameSlotDifference)){
					return true;
				}
				
				return false;
			}
			
	         /*
              * vergleich 
              * 
              */
			
			
}
            
             