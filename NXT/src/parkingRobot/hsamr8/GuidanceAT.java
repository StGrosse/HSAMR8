/**



        �nderungen:
        - Ausparken implementiert


*/


package parkingRobot.hsamr8;

import parkingRobot.INavigation.ParkingSlot; // zus�tzlich
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
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
import lejos.robotics.navigation.Pose;

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
	static float [] Koeffizienten = new float [4];
	static float [] Zielkoordinaten = new float [2];
    static float [] Startkoordinaten = new float [2];

	public enum CurrentStatus
	{
		DRIVING,
		INACTIVE,
		EXIT
	}

	private enum CurrentModus
	{
		SCOUT,          //
		PARK_NOW,       //
		PARK_THIS,      //
        EINPARKEN,      //
		AUSPARKEN,
		PAUSE
	}


	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatus currentStatus 	= CurrentStatus.INACTIVE;
	protected static CurrentModus currentModus 	= CurrentModus.PAUSE;
	protected static CurrentModus lastModus		= CurrentModus.PAUSE;


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

        currentStatus = CurrentStatus.INACTIVE;
        currentModus = CurrentModus.PAUSE;
        lastModus    = CurrentModus.PAUSE;


		NXTMotor leftMotor  = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.C);

		IMonitor monitor = new Monitor();

		IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
		//perception.calibrateLineSensors();

		INavigation navigation = new NavigationAT(perception, monitor);
		IControl    control    = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);
		INxtHmi  	hmi        = new HmiPLT(perception, navigation, control, monitor);


       

		monitor.startLogging();


while(true)
{
	//showData(navigation, perception);
	monitor.writeGuidanceComment("Modus: "+currentModus+" Status: "+currentStatus);

    switch( currentStatus )
    {
/*###########################################################################################################################*/
        case DRIVING:

            switch( currentModus )
            {

				case SCOUT:
					// MONITOR (example)
                    // monitor.writeGuidanceComment("Guidance_Driving");

					if( lastModus != CurrentModus.SCOUT )
                    {
						control.setCtrlMode(ControlMode.LINE_CTRL);
						navigation.setDetectionState(true);
					}

					lastModus = currentModus;

					if( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE || Button.ENTER.isDown())
					{
						currentStatus = CurrentStatus.INACTIVE;
						while(Button.ENTER.isDown())
						{
						    Thread.sleep(1);
                        }
					}

					else if(hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown() )
                    {
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown())
                        {
                            Thread.sleep(1);
                        }
					}

					else if(hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_NOW)
					{
						currentModus = CurrentModus.PARK_NOW;
					}

					else if(hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS)
                    {
                        currentModus = CurrentModus.PARK_THIS;
					}



                break;

/*###########################################################################################################################*/

                    case PARK_NOW:

                        if( lastModus != CurrentModus.PARK_NOW )
                        {
                            control.setCtrlMode(ControlMode.LINE_CTRL);
                            navigation.setDetectionState(true);
                        }

                        lastModus = currentModus;


                        if(pruefeAufLuecke(navigation.getPose().getX(), navigation.getPose().getY(), navigation, currentModus,hmi))   // Roboter befindet sich im Abstand d (d in pruefeAufLuecke definiert) von der Parkl�cke
                        {
                            currentStatus = CurrentStatus.DRIVING;
                            currentModus = CurrentModus.EINPARKEN;
                        }



                    if( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE || Button.ENTER.isDown())
					{
						currentStatus = CurrentStatus.INACTIVE;
						while(Button.ENTER.isDown())
						{
						    Thread.sleep(1);
                        }
					}

					else if(hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown() )
                    {
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown())
                        {
                            Thread.sleep(1);
                        }
					}

                    else if( hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS)
					{
						currentModus = CurrentModus.PARK_THIS;
					}

					else if( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT)
					{
						currentModus = CurrentModus.SCOUT;
					}

                    break;
/*###########################################################################################################################*/

                    case PARK_THIS:

                        if ( lastModus != CurrentModus.PARK_THIS )
                        {
                            control.setCtrlMode(ControlMode.LINE_CTRL);
                            navigation.setDetectionState(true);
                        }

                        lastModus = currentModus;


                        if(pruefeAufLuecke(navigation.getPose().getX(), navigation.getPose().getY(), navigation, currentModus,hmi))   // Roboter befindet sich im Abstand d (d in pruefeAufLuecke definiert) von der Parkl�cke
                        {
                            currentStatus = CurrentStatus.DRIVING;
                            currentModus = CurrentModus.EINPARKEN;
                        }



                        if( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE || Button.ENTER.isDown())
                        {
                            currentStatus = CurrentStatus.INACTIVE;
                            while(Button.ENTER.isDown())
                            {
                                Thread.sleep(1);
                            }
                        }

                        else if(hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown() )
                        {
                            currentStatus = CurrentStatus.EXIT;
                            while(Button.ESCAPE.isDown())
                            {
                                Thread.sleep(1);
                            }
                        }

                        else if( hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_NOW)
                        {
                            currentModus = CurrentModus.PARK_NOW;
                        }

                        else if( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT)
                        {
                            currentModus = CurrentModus.SCOUT;
                        }


                    break;
/*###########################################################################################################################*/

				case AUSPARKEN:

				    float xs, ys, xz, yz;

                        if ( lastModus != CurrentModus.AUSPARKEN )
                        {
                            xs = navigation.getPose().getX();
                            ys = navigation.getPose().getY();

                            xz = Startkoordinaten[0];    // noch vom Einparken
                            yz = Startkoordinaten[1];    // noch vom Einparken

                            control.setPath(Koeffizienten, true, navigation.getPose(), new Pose(xz, yz,0.0f));
                            control.setCtrlMode(ControlMode.PARK_CTRL);
                        }

                        lastModus = currentModus;

                        if(control.getParkStatus())    /** zur�ck zu PAUSE, wenn fertig mit Einparken*/
                            currentStatus = CurrentStatus.INACTIVE;


                        if( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE || Button.ENTER.isDown())
                        {
                            currentStatus = CurrentStatus.INACTIVE;
                            while(Button.ENTER.isDown())
                            {
                                Thread.sleep(1);
                            }
                        }

                        else if(hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown() )
                        {
                            currentStatus = CurrentStatus.EXIT;
                            while(Button.ESCAPE.isDown())
                            {
                                Thread.sleep(1);
                            }
                        }



                break;
/*###########################################################################################################################*/
                    case EINPARKEN: // Roboter steht vor L�cke und hat Zielkoordinaten f�r Einparken
                    	navigation.setDetectionState(false);
                        float xs1, ys1, xz1, yz1;

                        if ( lastModus != CurrentModus.EINPARKEN )
                        {
                            xs1 = navigation.getPose().getX();
                            ys1 = navigation.getPose().getY();
                            Startkoordinaten[0] = xs1;
                            Startkoordinaten[1] = ys1;

                            xz1 = Zielkoordinaten[0];
                            yz1 = Zielkoordinaten[1];

                            control.setPath(Pfadgenerator(xs1, ys1, xz1, yz1), false, navigation.getPose(), new Pose(xz1, yz1,0.0f));
                            control.setCtrlMode(ControlMode.PARK_CTRL);
                        }

                        lastModus = currentModus;

                        if(control.getParkStatus())    /** zur�ck zu PAUSE, wenn fertig mit Einparken*/
                            currentStatus = CurrentStatus.INACTIVE;


                        if( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE || Button.ENTER.isDown())
                        {
                            currentStatus = CurrentStatus.INACTIVE;
                            while(Button.ENTER.isDown())
                            {
                                Thread.sleep(1);
                            }
                        }

                        else if(hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown() )
                        {
                            currentStatus = CurrentStatus.EXIT;
                            while(Button.ESCAPE.isDown())
                            {
                                Thread.sleep(1);
                            }
                        }

					break;

/*###########################################################################################################################*/


			default:
				break;
        	} // switch case Modus ende

        break;

/***********************************************************************************************************************************************************************************/
        case INACTIVE:

            lastModus = CurrentModus.PAUSE;
            currentModus = CurrentModus.PAUSE;
        	control.setCtrlMode(ControlMode.INACTIVE);
			navigation.setDetectionState(false);

            if( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT || Button.ENTER.isDown() )
            {
            	currentStatus = CurrentStatus.DRIVING;

                if(control.getParkStatus())    /** wenn fertig mit Einparken*/
                    currentModus = CurrentModus.AUSPARKEN;

                else
                {
                    currentModus = CurrentModus.SCOUT;    
                }
                
                while(Button.ENTER.isDown())
                {
                    Thread.sleep(1);
                }
            }

            if( hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_NOW && !control.getParkStatus())
            {
                currentStatus = CurrentStatus.DRIVING;
                currentModus = CurrentModus.PARK_NOW;
            }

            if( hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS && !control.getParkStatus())
            {
                currentStatus = CurrentStatus.DRIVING;
                currentModus = CurrentModus.PARK_THIS;
            }

            else if(hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown() )
            {
				currentStatus = CurrentStatus.EXIT;
				while(Button.ESCAPE.isDown())
                {
                    Thread.sleep(1);
                }
            }

        break;
/***********************************************************************************************************************************************************************************/
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


    } // switch case Status ende

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
        y(x) = a*x� + b*x� + c*x + d
        */

        werte[0] = d;
        werte[1] = c;
        werte[2] = b;
        werte[3] = a;

        Koeffizienten[0] = werte[0];
        Koeffizienten[1] = werte[1];
        Koeffizienten[2] = werte[2];
        Koeffizienten[3] = werte[3];

        return werte;

	}

	private static boolean pruefeAufLuecke(float x, float y, INavigation navigation, CurrentModus modus, INxtHmi hmi) // Roboterkoordinaten werden �bergeben (in cm)
	{

        boolean erg = false;               // Initialisierung mit 0: "keine passende Parkl�cke gefunden"

        float d = 0.1f;               // 2*d = Abstandsintervall von Parkl�cke f�r Erkennung
        float a = 0.20f;               // Abstand von Parkl�ckenrand (ca 5cm + halbe Roboterl�nge)
        float b = 0.30f;               // Einparktiefe

	    float xp_f, xp_b, yp_f, yp_b, lp_x, lp_y;           // jeweilige Koordinaten bzw. L�ngen der Parkl�cken

        if(navigation.getParkingSlots() != null)
        {
            if( modus == CurrentModus.PARK_NOW)
            for(int i = 0; i < navigation.getParkingSlots().length; i++)
            {
                ParkingSlot slot = navigation.getParkingSlots()[i];
                if(slot.getStatus()==ParkingSlotStatus.GOOD)
                {

                	xp_f = slot.getFrontBoundaryPosition().x;
                	xp_b = slot.getBackBoundaryPosition().x;
                	yp_f = slot.getFrontBoundaryPosition().y;
                	yp_b = slot.getBackBoundaryPosition().y;

                	lp_x = xp_f - xp_b;
                	lp_y = yp_f - yp_b;
                	if(lp_x < 0) lp_x *=-1;
                	if(lp_y < 0) lp_y *=-1;

                	if(y < 0.15 && x < 1.60 && yp_f < 0.15 && xp_b < 1.60)        // Roboter und Parkl�cke auf unterer Linie
                	{
                    if( (xp_b-x) < d && (xp_b-x) > -d )
                    {
                        erg = true;
                        Zielkoordinaten[0] = xp_b + lp_x - a;
                        Zielkoordinaten[1] = y - b;
                    }
                }

                if(y > 0.15 && x < 1.60 && yp_f > 0.15 && xp_b < 1.60)        // Roboter und Parkl�cke auf oberer Linie
                {
                    if( (x-xp_b) < d && (x-xp_b) > -d )
                    {
                        erg = true;
                        Zielkoordinaten[0] = xp_b - lp_x + a;
                        Zielkoordinaten[1] = y + b;
                    }
                }

                if(x > 1.60 && xp_f > 1.60)                               // Roboter und Parkl�cke auf rechter Linie
                {
                    if( (yp_b-y) < d && (yp_b-y) > -d)
                    {
                        erg = true;
                        Zielkoordinaten[0] = yp_b + lp_y - a;
                        Zielkoordinaten[1] = x + b;
                    }
                }
                }
            }

            if( modus == CurrentModus.PARK_THIS)
            {
                ParkingSlot slot = navigation.getParkingSlots()[hmi.getSelectedParkingSlot()];

                xp_f = slot.getFrontBoundaryPosition().x;
                xp_b = slot.getBackBoundaryPosition().x;
                yp_f = slot.getFrontBoundaryPosition().y;
                yp_b = slot.getBackBoundaryPosition().y;

                lp_x = xp_f - xp_b;
                lp_y = yp_f - yp_b;
                if(lp_x < 0) lp_x *=-1;
                if(lp_y < 0) lp_y *=-1;

                if(y < 0.15 && x < 1.60 && yp_f < 0.15 && xp_b < 1.60)        // Roboter und Parkl�cke auf unterer Linie
                {
                    if( (xp_b-x) < d && (xp_b-x) > -d )
                    {
                        erg = true;
                        Zielkoordinaten[0] = xp_b + lp_x - a;
                        Zielkoordinaten[1] = y - b;
                    }
                }

                if(y > 0.15 && x < 1.60 && yp_f > 0.15 && xp_b < 1.60)        // Roboter und Parkl�cke auf oberer Linie
                {
                    if( (x-xp_b) < d && (x-xp_b) > -d )
                    {
                        erg = true;
                        Zielkoordinaten[0] = xp_b - lp_x + a;
                        Zielkoordinaten[1] = y + b;
                    }
                }

                if(x > 1.60 && xp_f > 1.60)                               // Roboter und Parkl�cke auf rechter Linie
                {
                    if( (yp_b-y) < d && (yp_b-y) > -d)
                    {
                        erg = true;
                        Zielkoordinaten[0] = yp_b + lp_y - a;
                        Zielkoordinaten[1] = x + b;
                    }
                }

            }


        }
        else
            erg = false; // Fehler


	    return erg;
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
