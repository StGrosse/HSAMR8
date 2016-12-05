package parkingRobot.hsamr8;

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
		PAUSE,
		SCOUT,
		PARK_NOW,
		PARK_THIS,
		DISCONNECT,

        EINPARKEN,
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
		NXTMotor rightMotor = new NXTMotor(MotorPort.A);

		IMonitor monitor = new Monitor();

		IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
		perception.calibrateLineSensors();

		INavigation navigation = new NavigationAT(perception, monitor);
		IControl    control    = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);
		INxtHmi  	hmi        = new HmiPLT(perception, navigation, control, monitor);

		monitor.startLogging();

		while(true)
            {
			showData(navigation, perception);

        	switch ( currentStatus )
        	{
/***********************************************************************************************************************************************************************************/
				case PAUSE:

                    if ( lastStatus != CurrentStatus.PAUSE )
                    {
						control.setCtrlMode(ControlMode.INACTIVE);
					}

					lastStatus = currentStatus;

					if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT || Button.ENTER.isDown())
					{
						currentStatus = CurrentStatus.SCOUT;
						while(Button.ENTER.isDown())
						{
						    Thread.sleep(1);
                        }
					}


					else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown() )
                    {
						currentStatus = CurrentStatus.DISCONNECT;
						while(Button.ESCAPE.isDown())
                        {
                            Thread.sleep(1);
                        }
					}

                    else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_NOW)
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
					// MONITOR (example)
                    // monitor.writeGuidanceComment("Guidance_Driving");

					if ( lastStatus != CurrentStatus.SCOUT )
                    {
                        navigation.setDetectionState(true);
						control.setCtrlMode(ControlMode.LINE_CTRL);
					}

					lastStatus = currentStatus;

					if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE || Button.ENTER.isDown())
					{
						currentStatus = CurrentStatus.PAUSE;
						while(Button.ENTER.isDown())
						{
						    Thread.sleep(1);
                        }
					}

					else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown() )
                    {
						currentStatus = CurrentStatus.DISCONNECT;
						while(Button.ESCAPE.isDown())
                        {
                            Thread.sleep(1);
                        }
					}

					navigation.setDetectionState(false);

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

                    if ( lastStatus != CurrentStatus.PARK_NOW )
                    {
						control.setCtrlMode(ControlMode.LINE_CTRL);
					}

					lastStatus = currentStatus;


					/**
                        Auf Parklücke prüfen
                        wenn Triang. Sensoren Parklücke erkannt haben  --> Einparken
					*/

                    if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown() )
                    {
						currentStatus = CurrentStatus.DISCONNECT;
						while(Button.ESCAPE.isDown())
                        {
                            Thread.sleep(1);
                        }
					}


                    if(true) /** (noch ändern) zurück zu PAUSE, wenn fertig mit PARK_NOW */
                    currentStatus = CurrentStatus.PAUSE;

                    break;
/***********************************************************************************************************************************************************************************/

                    case PARK_THIS:

                    if ( lastStatus != CurrentStatus.PARK_THIS )
                    {
						control.setCtrlMode(ControlMode.LINE_CTRL);
					}

					lastStatus = currentStatus;


					/**
                        Auf Parklücke prüfen
                        wenn Triang. Sensoren Parklücke erkannt haben  --> Einparken
					*/

                    if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown() )
                    {
						currentStatus = CurrentStatus.DISCONNECT;
						while(Button.ESCAPE.isDown())
                        {
                            Thread.sleep(1);
                        }
					}

                    if(true) /** (noch ändern) zurück zu PAUSE, wenn fertig mit PARK_THIS */
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

					/** Pfadgenerator.generiereWerte()      */
					/** Parke ein:      */

					if ( lastStatus != CurrentStatus.EINPARKEN )
                    {
						control.setCtrlMode(ControlMode.PARK_CTRL);
					}

					lastStatus = currentStatus;

					/****************************************/

					if(true) /** (noch ändern) zurück zu PAUSE, wenn fertig mit EINPARKEN */
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
