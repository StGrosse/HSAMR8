/**



        Änderungen:
        - Pfadgenerator implementiert (Berechnung der Polynomparameter, Übergabe mittels Array)
        - für PARK_NOW werden Parklücken erkannt und Zielkoordinaten für Einparken berechnet
        - geringfügige Erweiterung für EINPARKEN



*/


package parkingRobot.hsamr8;

import parkingRobot.INavigation.ParkingSlot; // zusätzlich

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
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


public class GuidanceAT
{

	/**
	 * states for the main finite state machine. This main states are requirements because they invoke different
	 * display modes in the human machine interface.
	 */
	public enum CurrentStatus
	{
		PAUSE,          //
		SCOUT,          //
		PARK_NOW,       //
		PARK_THIS,
		DISCONNECT,     //
        EINPARKEN,      //
		AUSPARKEN
	}


	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatus currentStatus 	= CurrentStatus.PAUSE;
	/**
	 * state in which the main finite state machine was running before entering the actual state
	 */
	protected static CurrentStatus lastStatus		= CurrentStatus.PAUSE;


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


	/**
	 * main method of project 'ParkingRobot'
	 *
	 * @param args standard string arguments for main method
	 * @throws Exception exception for thread management
	 */
	public static void main(String[] args) throws Exception
	{

        currentStatus = CurrentStatus.PAUSE;
        lastStatus    = CurrentStatus.DISCONNECT;

		NXTMotor leftMotor  = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.C);

		IMonitor monitor = new Monitor();

		IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
		//perception.calibrateLineSensors();

		INavigation navigation = new NavigationAT(perception, monitor);
		IControl    control    = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);
		INxtHmi  	hmi        = new HmiPLT(perception, navigation, control, monitor);


        float [] Zielkoordinaten = new float [2];

		monitor.startLogging();

		while(true)
            {
			showData(navigation, perception);

        	switch( currentStatus )
        	{
/***********************************************************************************************************************************************************************************/
				case PAUSE:
					LCD.clear();
					LCD.drawString("pause", 0, 0);

                    if ( lastStatus != CurrentStatus.PAUSE )
                    {
						control.setCtrlMode(ControlMode.INACTIVE);
						navigation.setDetectionState(false);
					}

					lastStatus = currentStatus;

					if( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT || Button.ENTER.isDown())
					{
						currentStatus = CurrentStatus.SCOUT;
						while(Button.ENTER.isDown())
						{
						    Thread.sleep(1);
                        }
					}


					else if(hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown() )
                    {
						currentStatus = CurrentStatus.DISCONNECT;
						while(Button.ESCAPE.isDown())
                        {
                            Thread.sleep(1);
                        }
					}

                    else if(hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_NOW)
					{
					    lastStatus = CurrentStatus.PARK_NOW;
						currentStatus = CurrentStatus.AUSPARKEN;
					}

					else if(hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS)
                    {
                        lastStatus = CurrentStatus.PARK_THIS;
						currentStatus = CurrentStatus.AUSPARKEN;
					}

                break;

/***********************************************************************************************************************************************************************************/

				case SCOUT:
					LCD.clear();
					LCD.drawString("scout",0,0);
					// MONITOR (example)
                    // monitor.writeGuidanceComment("Guidance_Driving");

					if( lastStatus != CurrentStatus.SCOUT )
                    {
                        //zum Test von setPose:
						//navigation.setDetectionState(false);
						
						navigation.setDetectionState(true);
						control.setCtrlMode(ControlMode.LINE_CTRL);
					}

					lastStatus = currentStatus;

					if( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE || Button.ENTER.isDown())
					{
						currentStatus = CurrentStatus.PAUSE;
						while(Button.ENTER.isDown())
						{
						    Thread.sleep(1);
                        }
					}

					else if(hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown() )
                    {
						currentStatus = CurrentStatus.DISCONNECT;
						while(Button.ESCAPE.isDown())
                        {
                            Thread.sleep(1);
                        }
					}
					else if(hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS){
						currentStatus = CurrentStatus.PARK_NOW;
					}



                break;
/***********************************************************************************************************************************************************************************/
				case DISCONNECT:
					hmi.disconnect();
					/** NOTE: RESERVED FOR FUTURE DEVELOPMENT (PLEASE DO NOT CHANGE)
					// monitor.sendOfflineLog();
					*/
					monitor.stopLogging();
					System.exit(0);
                break;

/***********************************************************************************************************************************************************************************/

                    case PARK_NOW:
                    	
                    	LCD.clear();
                    	LCD.drawString("park now", 0, 0);
                        float [] werte = new float[3];

                        if( lastStatus != CurrentStatus.PARK_NOW )
                        {
                            control.setCtrlMode(ControlMode.LINE_CTRL);
                            navigation.setDetectionState(true);//WICHTIG
                        }

                        lastStatus = currentStatus;


                        werte = pruefeAufLuecke(navigation.getPose().getX()*100, navigation.getPose().getY()*100, navigation);
                        if(werte[0] == 1)   // Roboter befindet sich im Abstand d (d in pruefeAufLuecke definiert) von der Parklücke
                        {
                            Zielkoordinaten[0] = werte[1];
                            Zielkoordinaten[1] = werte[2];
                            //currentStatus = CurrentStatus.EINPARKEN;
                            currentStatus = CurrentStatus.PAUSE; // zum Testen: Roboter bleibt vor Parklücke stehen
                        }



                        if(hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown() )
                        {
                            currentStatus = CurrentStatus.DISCONNECT;
                            while(Button.ESCAPE.isDown())
                            {
                                Thread.sleep(1);
                            }
                        }

                        if( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE)
                        {
                            currentStatus = CurrentStatus.PAUSE;
                        }

                    break;
/***********************************************************************************************************************************************************************************/

                    case PARK_THIS:
                    	
                    	LCD.clear();
                    	LCD.drawString("park this", 0, 0);
                        if ( lastStatus != CurrentStatus.PARK_THIS )
                        {
                            control.setCtrlMode(ControlMode.LINE_CTRL);
                            navigation.setDetectionState(true);//WICHTIG
                        }
                        werte = pruefeAufLuecke(navigation.getPose().getX()*100, navigation.getPose().getY()*100, navigation);
                        if(werte[0] == 1)   // Roboter befindet sich im Abstand d (d in pruefeAufLuecke definiert) von der Parklücke
                        {
                            Zielkoordinaten[0] = werte[1];
                            Zielkoordinaten[1] = werte[2];
                            //currentStatus = CurrentStatus.EINPARKEN;
                            currentStatus = CurrentStatus.PAUSE; // zum Testen: Roboter bleibt vor Parklücke stehen
                        }
                        lastStatus = currentStatus;


                        /**
                            prüfe auf Abstand zu Zielparklücke (ähnlich wie bei PARK_NOW: pruefeAufLuecke)
                            wenn angekommen  --> Einparken
                        */


                        if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown() )
                        {
                            currentStatus = CurrentStatus.DISCONNECT;
                            while(Button.ESCAPE.isDown())
                            {
                                Thread.sleep(1);
                            }
                        }

                        if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE)
                        {
                            currentStatus = CurrentStatus.PAUSE;
                        }

                        if(true) /** (noch ändern) zurück zu PAUSE, wenn fertig mit PARK_THIS (bei EINPARKEN implementiert)*/
                        currentStatus = CurrentStatus.PAUSE;

                    break;
/***********************************************************************************************************************************************************************************/

				case AUSPARKEN:

				    /*********AUSPARKEN()!!!!!!!!!!!!!!!!!!**********/


                    if(lastStatus == CurrentStatus.PARK_NOW)
                    {
                        currentStatus = CurrentStatus.PARK_NOW;
                    }

                    else if(lastStatus == CurrentStatus.PARK_THIS)
                    {
                        currentStatus = CurrentStatus.PARK_THIS;
                    }

                    lastStatus = CurrentStatus.AUSPARKEN;

                break;
/***********************************************************************************************************************************************************************************/

                    case EINPARKEN:

                        float xs, ys, xz, yz;

                        xs = navigation.getPose().getX()*100;
                        ys = navigation.getPose().getY()*100;

                        xz = Zielkoordinaten[0];
                        yz = Zielkoordinaten[1];


                        if ( lastStatus != CurrentStatus.EINPARKEN )
                        {
                            control.setCtrlMode(ControlMode.PARK_CTRL);
                            /** control.setCtrlMode(ControlMode.PARK_CTRL(Pfadgenerator(xs, ys, xz, yz)));      */
                        }

                        lastStatus = currentStatus;

                        /****************************************/

                        if(true)    /** (noch ändern) zurück zu PAUSE, wenn fertig mit Einparken    */
                                    /** z.B. if(control.getParkStatus == true)                      */
                            currentStatus = CurrentStatus.PAUSE;

					break;

/***********************************************************************************************************************************************************************************/



			default:
				break;
        	} // switch case ende

        	Thread.sleep(100);




		} //while(true) ende





	}  // main ende


	/**
	 * returns the actual state of the main finite state machine as defined by the requirements
	 *
	 * @return actual state of the main finite state machine
	 */
	public static CurrentStatus getCurrentStatus()
	{
		return GuidanceAT.currentStatus;
	}

	private static float[] Pfadgenerator(float xs, float ys, float xz, float yz)
	{
	    float []werte=new float[4];

	    float a, b, c, d;
	    float x1, x2, x3, x4, x5, x6, x7, x8, x9, x10;
	    float y1, y2;
	    float x11, x12, x13, x14;
	    float y11, y12;

        x1  = 1 / (xs);
        x2  = 1 / (xs * xs);
        x3  = 1 / (xs * xs * xs);
        x4  = 1 / (xz);
        x5  = 1 / (xz * xz);
        x6  = 1 / (xz * xz * xz);
        x7  = 2 / (3 * xs);
        x8  = 1 / (3 * xs * xs);
        x9  = 2 / (3 * xz);
        x10 = 1 / (3 * xz * xz);
        y1  = ys / (xs * xs * xs);
        y2  = yz / (xz * xz * xz);
        x11 = (x8-x2)/(x7-x1) - (x5-x2)/(x4-x1);
        x12 = (-x3)/(x7-x1) - (x6-x3)/(x4-x1);
        x13 = (x10-x2)/(x9-x1) - (x5-x2)/(x4-x1);
        x14 = (-x3)/(x9-x1) - (x6-x3)/(x4-x1);
        y11 = (-y1)/(x7-x1) - (y2-y1)/(x4-x1);
        y12 = (-y1)/(x9-x1) - (y2-y1)/(x4-x1);

        d = (y12/x13 - y11/x11)/(x14/x13 - x12/x11);
        c = y11/x11 - d*x12/x11;
        b = (y2-y1)/(x4-x1) - c*(x5-x2)/(x4-x1) - d*(x6-x3)/(x4-x1);
        a = y1 - b*x1 - c*x2 - d*x3;

        /**
        y(x) = a*x³ + b*x² + c*x + d
        */

        werte[0] = a;
        werte[1] = b;
        werte[2] = c;
        werte[3] = d;

        return werte;

	}

	private static float[] pruefeAufLuecke(float x, float y, INavigation navigation) // Roboterkoordinaten werden übergeben (in cm)
	{
	    float [] werte = new float [3];     // werte[0] --> Parklücke vorhanden(1)/nicht(0)
                                    // werte[1] --> Ziel X - Koordinate für Einparken wenn werte[0] == 1
                                    // werte[2] --> Ziel Y - Koordinate für Einparken wenn werte[0] == 1

        werte[0] = 0;               // Initialisierung mit 0: "keine passende Parklücke gefunden"

        float d = 10;               // 2*d = Abstandsintervall von Parklücke für Erkennung
        float a = 20;               // Abstand von Parklückenrand (ca 5cm + halbe Roboterlänge)
        float b = 30;               // Einparktiefe

	    float xp_f, xp_b, yp_f, yp_b, lp_x, lp_y;           // jeweilige Koordinaten bzw. Längen der Parklücken

        if(navigation.getParkingSlots() != null)
        {
            for(int i = 0; i < navigation.getParkingSlots().length; i++)
            {
                ParkingSlot slot = navigation.getParkingSlots()[i];

                xp_f = slot.getFrontBoundaryPosition().x * 100;
                xp_b = slot.getBackBoundaryPosition().x  * 100;
                yp_f = slot.getFrontBoundaryPosition().y * 100;
                yp_b = slot.getBackBoundaryPosition().y  * 100;

                lp_x = xp_f - xp_b;
                lp_y = yp_f - yp_b;
                if(lp_x < 0) lp_x *=-1;
                if(lp_y < 0) lp_y *=-1;

                if(y < 15 && x < 160 && yp_f < 15 && xp_b < 160)        // Roboter und Parklücke auf unterer Linie
                {
                    if( (xp_b-x) < d && (xp_b-x) > -d )
                    {
                        werte[0] = 1;
                        werte[1] = xp_b + lp_x - a;
                        werte[2] = y - b;
                    }
                }

                if(y > 15 && x < 160 && yp_f > 15 && xp_b < 160)        // Roboter und Parklücke auf oberer Linie
                {
                    if( (x-xp_b) < d && (x-xp_b) > -d )
                    {
                        werte[0] = 1;
                        werte[1] = xp_b - lp_x + a;
                        werte[2] = y + b;
                    }
                }

                if(x > 160 && xp_f > 160)                               // Roboter und Parklücke auf rechter Linie
                {
                    if( (yp_b-y) < d && (yp_b-y) > -d)
                    {
                        werte[0] = 1;
                        werte[1] = x + b;
                        werte[2] = yp_b + lp_y - a;
                    }
                }

            }

        }
        else
            werte[0] = -1; // Fehler


	    return werte;
	}


	/**
	 * plots the actual pose on the robots display
	 *
	 * @param navigation reference to the navigation class for getting pose information
	 */
	protected static void showData(INavigation navigation, IPerception perception)
	{
		LCD.clear();

		LCD.drawString("X (in cm): " + (navigation.getPose().getX()*100), 0, 0);
		LCD.drawString("Y (in cm): " + (navigation.getPose().getY()*100), 0, 1);
		LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading()/Math.PI*180), 0, 2);

    //	perception.showSensorData();

    //    	if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
    //			LCD.drawString("HMI Mode SCOUT", 0, 3);
    //		}else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
    //			LCD.drawString("HMI Mode PAUSE", 0, 3);
    //		}else{
    //			LCD.drawString("HMI Mode UNKNOWN", 0, 3);
    //		}
	}
}
