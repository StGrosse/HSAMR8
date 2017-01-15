package parkingRobot.hsamr8;

import java.util.ArrayList;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.nxt.Sound;
import lejos.robotics.navigation.Pose;
import lejos.nxt.LCD;

import parkingRobot.INavigation;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;


import parkingRobot.hsamr8.NavigationThread;
import parkingRobot.hsamr8.GuidanceAT;


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

	static final double encoderError=0.025;
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
	/**
	 * distance from frontsidesensor to Roboter
	 */
	double parkenGutLength = 0.045;

	/**
	 * distance from backsidesensor to Roboter
	 */
	double parkenGutLength2 = 0.08;
    
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
	 * In line 4 error must be remove
	 */

	//double errorX=0.05;

	
	
	
	
	/**
	 * parkinSlotsuchen parameter
	 */
	/**
	 * which line the robot is on
	 */
	int currentLine;
	/**
	 * initialize the Array so the Pad could see the parkingSlot 
	 */
	INavigation.ParkingSlot ParkingSlot = null;
	INavigation.ParkingSlot Slots[] = null;
	ArrayList<ParkingSlot> slotList = new ArrayList<ParkingSlot>();
	/**
	 * every parkingSlot have a ID
	 */
	int parken_ID=0; 
	int parkenSlot_ID=0;
	
	/**
	 * a signal the back boundary has already been found or not
	 */
	boolean parkingSuchen=false;
	/**
	 *  initialize the backBoundry position and the frontBoundry position
	 */
	Point backBoundaryPosition = null;
	Point frontBoundaryPosition = null;
	/**
	 * die measurementQuality with the robot
	 */
	int measurementQuality;
    /**
     * status with parkingSlot in line0, line1 and line4
     */
	boolean parkenSlotSuch0 = false;
	boolean parkenSlotSuch1 = false;
	boolean parkenSlotSuch4 = false;
	/**
	 * signal whether parkingSlot have been found or not
	 */
	boolean signal=false;
	
	
	double frontparkPosition = 0;
	double backparkPosition = 0;
	/**
	 *  parkingSlot's length
	 */
	double parkingSlotLength=0;
	/**
	 * die circle the Robot already run
	 */
	int round=0;
	/**
	 * the parameter SameSlotDifference 8 cm
	 */
	private static final double SameSlotDifference =0.08;
	/**
	 * returns the map
	 */
	public Line[] getMap() {
		return map;
	}
	
	
	
	
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
		monitor.writeNavigationVar("X_wert", "" +  this.getPose().getX());
		monitor.writeNavigationVar("Y_wert", "" +  this.getPose().getY());
		monitor.writeNavigationVar("Alpha_wert", "" +  this.getPose().getHeading());
		
		if (this.parkingSlotDetectionIsOn) 
			try{
				this.detectParkingSlot();
			}
			catch(NullPointerException ne){
				LCD.clear();
				LCD.drawString("nullpointer", 0, 0);
			}

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
		
		this.monitor.writeNavigationComment("frontSide"+this.frontSideSensorDistance);
		this.monitor.writeNavigationComment("backSide"+this.backSideSensorDistance);
	}		 	
	
	/**
	 * calculates the robot pose from the encoder measurements 
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
				xResult = 0.0;
				yResult = 0.0;
				currentLine=0;
				round++;
			}
		}
     //	errorX=Math.abs(this.getPose().getX()-xResult)*encoderError;
	//	errorY=Math.abs(this.getPose().getX()-xResult)*encoderError;
		//line0
		if((this.pose.getHeading() < Math.PI * 0.10)&& (this.pose.getHeading() >- Math.PI * 0.10)){
		         currentLine=0;
				}
				
			
			
		if(currentLine==0){
			monitor.writeNavigationComment("enter loopline0");
			if (parkingSlotDetectionIsOn) {
			if ((this.pose.getX() >0.03)&& (this.pose.getX()<1.70)) {
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
			currentLine=0;
		}
		}
		
		
		//Line=1
		if ((this.pose.getHeading() >Math.PI * 0.44) && (this.pose.getHeading() < Math.PI * 0.55)&& (currentLine == 0)) {
			monitor.writeNavigationComment("enter line1");
			xResult=1.80;
			angleResult=0.5*Math.PI;
            currentLine=1;
          
			
		}
		
		if (currentLine== 1) {
            monitor.writeNavigationComment("enterloop1");
			currentLine=1;
			   //Einpaken
	        if (parkingSlotDetectionIsOn) {
				if ((this.pose.getY() > 0.05) && (this.pose.getY() < 0.55)) {
					if ((this.pose.getHeading() < Math.PI * 0.49) || (this.pose.getHeading() >Math.PI * 0.51)) {
						angleResult =0.5*Math.PI;
					}
					if (this.pose.getX() >1.81) {
						xResult = 1.80;
					}
					if (this.pose.getX() <1.79) {
						xResult = 1.80;
						 monitor.writeNavigationComment("enterloop1.1.80");

					}
					
				}
				currentLine = 1;
		   }

			

		}
		/*
		 * line 2
		 */

		if ((this.pose.getHeading() > Math.PI * 0.85) && (this.pose.getHeading() < Math.PI * 1.136)&& (currentLine== 1)) {
			monitor.writeNavigationComment("enter line2");
			yResult = 0.60;
			angleResult = Math.PI;
			currentLine = 2;
		}
		if (currentLine == 2) {
              currentLine=2;
              monitor.writeNavigationComment("enter loopline2");

		   //if (parkingSlotDetectionIsOn) {

				    monitor.writeNavigationComment("enter loopline2");
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
	
			
		}
		//Linien 4 noch Problem(Y(cm))
		if (currentLine == 4) {
			monitor.writeNavigationComment("enter loopline4");
              currentLine=4;
		   	if (parkingSlotDetectionIsOn) {
				
				if ((this.pose.getX() <1.65) && (this.pose.getX() >0.45)) {
			
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
			}

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
			if ((this.pose.getY() > 0.15) && (this.pose.getY() < 0.58)) {
				
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
				if (this.pose.getX() <1.75) {
					//parkluecke0 und 1
					monitor.writeNavigationComment("parkenLine0");
				   if ((parkenSlotSuch0 == false) && (this.frontSideSensorDistance >280)) {
						backBoundaryPosition = new Point((float) (this.pose.getX()), this.pose.getY());
						monitor.writeNavigationComment("back");
							parkenSlotSuch0 = true;
							Sound.beep();

					}
					if ((parkenSlotSuch0 == true) && (this.frontSideSensorDistance <170)) {
						frontBoundaryPosition =new Point((float) (this.pose.getX()-parkenGutLength), this.pose.getY());
						parkenSlotSuch0 = false;
						signal = true;
                        monitor.writeNavigationComment("front");
                        Sound.beep();
					}
				}
				break;
			case 1:
				if ((this.pose.getY() > 0.05) && (this.pose.getY() < 0.6)) {
					monitor.writeNavigationComment("parkenLine1");
					if ((parkenSlotSuch1 == false) && (this.frontSideSensorDistance > 320)) {
						backBoundaryPosition = new Point(this.pose.getX(), (float) (this.pose.getY() ));
						monitor.writeNavigationComment("back_Line1");
						parkenSlotSuch1 = true;
						Sound.beep();
						
				}
					if ((parkenSlotSuch1 == true) && (this.frontSideSensorDistance < 200)) {
						frontBoundaryPosition = new Point(this.pose.getX(),(float) (this.pose.getY()+parkenGutLength));
						//frontBoundaryPosition = new Point(this.pose.getX(),this.pose.getY());
						monitor.writeNavigationComment("front_Line1");
						parkenSlotSuch1 = false;
						signal = true;
						Sound.beep();
					}
				}
				break;
			
			case 4:
				if ((this.pose.getX() <1.2) && (this.pose.getX() >0.4)) {
					monitor.writeNavigationComment("parkenLine4");
					if ((parkenSlotSuch4 == false) && (this.backSideSensorDistance > 380)) {
						monitor.writeNavigationComment("back_Line4");
						backBoundaryPosition = new Point((float) (this.pose.getX()+parkenGutLength2), this.pose.getY());
						//backBoundaryPosition = new Point(this.pose.getX() , this.pose.getY());
						//parkenSlotSuch4 = true;
						parkenSlotSuch4 = true;
						Sound.beep();
			
				}
					if ((parkenSlotSuch4 == true) && (this.backSideSensorDistance < 120)) {
						monitor.writeNavigationComment("front_Line4");
						frontBoundaryPosition =new Point((float) (this.pose.getX()+parkenGutLength2), this.pose.getY());
						parkenSlotSuch4= false;
						signal = true;
						Sound.beep();
					}
				}
				break;
			}

			if (signal == true) {
				if ((this.frontBoundaryPosition != null) && (this.backBoundaryPosition != null)) {
					frontparkPosition = Math.abs(this.frontBoundaryPosition.getX()) + Math.abs(this.frontBoundaryPosition.getY());
					backparkPosition = Math.abs(this.backBoundaryPosition.getX()) + Math.abs(this.backBoundaryPosition.getY());
					parkingSlotLength = Math.abs(frontparkPosition - backparkPosition);
					parken_ID++;
					
					//set measurementquality
					measurementQuality=qualitySlot(parkingSlotLength,parken_ID-1);
					monitor.writeNavigationComment("line1");
				}

				if (parkingSlotLength>= 0.45) {
					ParkingSlotStatus status = ParkingSlotStatus.GOOD;
				    parkenSlot_ID = parken_ID-1;
				    ParkingSlot getParkingSlots = new ParkingSlot(parkenSlot_ID, backBoundaryPosition,
							frontBoundaryPosition,status, measurementQuality);
		          // Sound.beep();
				 //Aktualisierung der Parkluecken 
				  
				 if(round==0){
					    slotList.add(getParkingSlots);
			            Slots = new ParkingSlot[slotList.size()];
			            Slots = slotList.toArray(Slots);  //create a new array
				  }
				  
			    if(round>=1){
			    		
				    	for(int i=0; i<Slots.length-1;i++){
				    		if(i==20)
				    			break;			    			
				 
				    		try{
				    		if(sameSlot(Slots[i],getParkingSlots)){
				    			   if(Slots[i].getStatus()==ParkingSlotStatus.GOOD){
				    				   Point newBackBoundry=new Point((float)(Slots[i].getBackBoundaryPosition().getX()+getParkingSlots.getBackBoundaryPosition().getX())/2);
									   Point newFrontBoundry=new Point((float)(Slots[i].getFrontBoundaryPosition().getX()+getParkingSlots.getFrontBoundaryPosition().getX())/2);
									   getParkingSlots.setBackBoundaryPosition(newBackBoundry);
									   getParkingSlots.setFrontBoundaryPosition(newFrontBoundry);
									   slotList.set(i, getParkingSlots);
									   slotList.add(getParkingSlots);
								       Slots = new ParkingSlot[slotList.size()];
								       Slots = slotList.toArray(Slots);
								       break;
								      
				    			   }else{
				    				   slotList.set(i, getParkingSlots); 
				    				   slotList.add(getParkingSlots);
								       Slots = new ParkingSlot[slotList.size()];
								       Slots = slotList.toArray(Slots);
								       break;
				    			   }
				    			
				    		}
				    		else{
				    			slotList.add(getParkingSlots);
				    	  		Slots = new ParkingSlot[slotList.size()];
				    	   		Slots = slotList.toArray(Slots);  //create a new array 
				    	   		break;
				    		}
				    		}catch(NullPointerException d){
				    			LCD.drawString("nullpointer for", 0, 5);
				    			return;
				    		}
				    		
				    	}
			    }
				  
				  
                     
				} else if ((parkingSlotLength > 0) && (parkingSlotLength < 0.45)) {
					ParkingSlotStatus status = ParkingSlotStatus.BAD;
					parkenSlot_ID=parken_ID-1;
					
					ParkingSlot getParkingSlots = new ParkingSlot(parkenSlot_ID, backBoundaryPosition,
							frontBoundaryPosition,status, measurementQuality);
					 if(round==0){
						  slotList.add(getParkingSlots);
				          Slots = new ParkingSlot[slotList.size()];
				          Slots = slotList.toArray(Slots);  //create a new array
					  }
					
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
		if((((float)(Math.abs((x.getBackBoundaryPosition().getX()-y.getBackBoundaryPosition().getX())))<SameSlotDifference)
				&&(((float)(Math.abs(x.getBackBoundaryPosition().getY()-y.getBackBoundaryPosition().getY())))<SameSlotDifference))
				||(((float)(Math.abs(x.getFrontBoundaryPosition().getX()-y.getFrontBoundaryPosition().getX())))<SameSlotDifference)
				&&(((float)(Math.abs(x.getFrontBoundaryPosition().getY()-y.getFrontBoundaryPosition().getY())))<SameSlotDifference)){
			return true;
		}
		
		return false;
	}
			
			
	         /*
              * Compared unbekannter parkenSlot to parkenSlot  
              * @return true: paringSlotLengthX>=parkingSlotLengthY
              */
			public boolean compareSlot(ParkingSlot x, ParkingSlot y){
				//if(x.getMeasurementQuality()==5&&y.getMeasurementQuality()==5){
					double frontparkPositionX = Math.abs(x.getFrontBoundaryPosition().getX()) + Math.abs(x.getFrontBoundaryPosition().getY());
					double backparkPositionX = Math.abs(x.getBackBoundaryPosition().getX()) + Math.abs(x.getBackBoundaryPosition().getY());
					double frontparkPositionY = Math.abs(y.getFrontBoundaryPosition().getX()) + Math.abs(y.getFrontBoundaryPosition().getY());
					double backparkPositionY = Math.abs(y.getBackBoundaryPosition().getX()) + Math.abs(y.getBackBoundaryPosition().getY());
					double parkingSlotLength1 = Math.abs(frontparkPositionX - backparkPositionX);
					double parkingSlotLength2 = Math.abs(frontparkPositionY - backparkPositionY);
			//		if(x.getMeasurementQuality()>=y.getMeasurementQuality()){
					if(parkingSlotLength1>=parkingSlotLength2){
						return true;
					   }
					 else{
						return false;
					   }	
		//			}
									
				//}
				
			}
			  /*
             * ComparedparkingSlotlaenge  
             * @return true:neu vermessend Parkenluecke>=parkingSlotLengthx
             */
			public boolean langeSlot(double parkingSlotLength, ParkingSlot x){
					double frontparkPositionX = Math.abs(x.getFrontBoundaryPosition().getX()) + Math.abs(x.getFrontBoundaryPosition().getY());
					double backparkPositionX = Math.abs(x.getBackBoundaryPosition().getX()) + Math.abs(x.getBackBoundaryPosition().getY());
					double parkingSlotLength1 = Math.abs(frontparkPositionX - backparkPositionX);
					if(parkingSlotLength>=parkingSlotLength1&& x.getStatus()==ParkingSlotStatus.GOOD){
						return true;
					   }
					 else{
						return false;
					   }	
				
			}
			/*
			 * Die Qualitaet von Parkenluecken(die Genauigkeit)
			 * Array wahrenWert[] means true data
			 * Die Einheit ist cm
			 * @return number. Low value means low quality
			 */
			 public int qualitySlot(double parkingSlotLength, int parkenSlot_ID){
				 double[] wahrenWert= new double[4];
				 wahrenWert[0]=0.46;
				 wahrenWert[1]=0.58;
				 wahrenWert[2]=0.45;
				 wahrenWert[3]=0.64;
			//	double frontparkPosition = Math.abs(x.getFrontBoundaryPosition().getX()) + Math.abs(x.getFrontBoundaryPosition().getY());
		   //		double backparkPosition = Math.abs(x.getBackBoundaryPosition().getX()) + Math.abs(x.getBackBoundaryPosition().getY());
			//	double parkingSlotLength = Math.abs(frontparkPosition - backparkPosition);
				switch(parkenSlot_ID){
				case 0:
					  if(Math.abs(parkingSlotLength-wahrenWert[0])<=0.15){
						  measurementQuality=3;
					  }
					  else if(Math.abs(parkingSlotLength-wahrenWert[0])<0.25&&(Math.abs(parkingSlotLength-wahrenWert[0])>0.15)){
						  measurementQuality=2;
					  }
					  else{
						  measurementQuality=1;
					  }
					  break;
				case 1:
					  if(Math.abs(parkingSlotLength-wahrenWert[1])<=0.15){
						  measurementQuality=3;
					  }
					  else if(Math.abs(parkingSlotLength-wahrenWert[1])<0.25&&(Math.abs(parkingSlotLength-wahrenWert[1])>0.15)){
						  measurementQuality=2;
					  }
					  else{
						  measurementQuality=1;
					  }
					  break;
				case 2:
					  if(Math.abs(parkingSlotLength-wahrenWert[2])<=0.15){
						  measurementQuality=3;
					  }
					  else if(Math.abs(parkingSlotLength-wahrenWert[2])<0.25&&(Math.abs(parkingSlotLength-wahrenWert[2])>0.15)){
						  measurementQuality=2;
					  }
					  else{
						  measurementQuality=1;
					  }
					  break;
				case 3:
					  if(Math.abs(parkingSlotLength-wahrenWert[3])<=0.15){
						  measurementQuality=3;
					  }
					  else if(Math.abs(parkingSlotLength-wahrenWert[3])<0.25&&(Math.abs(parkingSlotLength-wahrenWert[3])>0.15)){
						  measurementQuality=2;
					  }
					  else{
						  measurementQuality=1;
					  }
					  break;
				}
				return measurementQuality;
			 }
			
			
}
         
            
             