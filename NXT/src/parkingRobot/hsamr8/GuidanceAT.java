
package parkingRobot.hsamr8;

import parkingRobot.INavigation.ParkingSlot;
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

/**
 * 	Hauptklasse Guidance - HS AMR
 */
public class GuidanceAT
{

	static double[] Koeffizienten = new double[4];	// Polynomkoeffizienten
	static double[] Zielkoordinaten = new double[2];	// allgemeine Zielkoordinaten f�r Ein-/Ausparken
	static double[] Startkoordinaten = new double[2];	// allgemeine Startkoordinaten f�r Ein-/Ausparken
	static int line=0;								// zum Speichern der aktuellen Linie

	/**
	 * 	Roboterstatus Enumeration f�r sp�teres Abfragen aus dem HMI Modul
	 * 	spiegelt aktuellen Roboterstatus wieder, je nach Modus
	 */
	public enum CurrentStatus	// Roboterstatus
	{
		DRIVING, INACTIVE, EXIT
	}

	private enum CurrentModus // Robotermodi
	{
		SCOUT, PARK_NOW, PARK_THIS, EINPARKEN, AUSPARKEN, PAUSE, DISCONNECT
	}

	protected static CurrentStatus currentStatus = CurrentStatus.INACTIVE;	// aktueller Status
	protected static CurrentModus currentModus = CurrentModus.PAUSE;		// aktueller Modus
	protected static CurrentModus lastModus = CurrentModus.PAUSE;			// zuletzt verwendeter Modus

	/**
	 * one line of the map of the robot course. The course consists of a closed
	 * chain of straight lines. Thus every next line starts where the last line
	 * ends and the last line ends where the first line starts. This
	 * documentation for line0 hold for all lines.
	 */
	static Line line0 = new Line(0  , 0 , 180, 0 );
	static Line line1 = new Line(180, 0 , 180, 60);
	static Line line2 = new Line(180, 60, 150, 60);
	static Line line3 = new Line(150, 60, 150, 30);
	static Line line4 = new Line(150, 30, 30 , 30);
	static Line line5 = new Line(30 , 30, 30 , 60);
	static Line line6 = new Line(30 , 60, 0  , 60);
	static Line line7 = new Line(0  , 60, 0  , 0 );
	/**
	 * map of the robot course. The course consists of a closed chain of
	 * straight lines. Thus every next line starts where the last line ends and
	 * the last line ends where the first line starts. All above defined lines
	 * are bundled in this array and to form the course map.
	 */
	static Line[] map = { line0, line1, line2, line3, line4, line5, line6, line7 };

	/**
	 * main method of project 'ParkingRobot'
	 *
	 * @param args
	 *            standard string arguments for main method
	 * @throws Exception
	 *             exception for thread management
	 */

	public static void main(String[] args) throws Exception
	{

		currentStatus = CurrentStatus.INACTIVE;	// Start: Inactivestatus
		currentModus  = CurrentModus.PAUSE;		// Start: Pausemodus
		lastModus = CurrentModus.PAUSE;			// Start: Pausemodus

		// Objekte anlegen:
		NXTMotor leftMotor = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.C);
		IMonitor monitor = new Monitor();
		IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
		INavigation navigation = new NavigationAT(perception, monitor);
		IControl control = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);
		INxtHmi hmi = new HmiPLT(perception, navigation, control, monitor);
		// perception.calibrateLineSensors();

		monitor.startLogging();

		while (true) 	// Endlosschleife
		{
			// showData(navigation, perception);

			switch (currentModus)
			{
			/********************************************************************************************************************************************************/
			case SCOUT: // wenn Modus Scout

				if (lastModus != CurrentModus.SCOUT)		// Initialbefehle (nur bei erstmaligen Aufruf)
				{
					control.setCtrlMode(ControlMode.LINE_CTRL);		// Linienverfolgung an
					navigation.setDetectionState(true);				// Parkl�ckendetection an
					currentStatus = CurrentStatus.DRIVING;			// Status Driving f�r Tablet
				}

				lastModus = currentModus;		// damit Initialbefehle nur ein mal aufgerufen werden

				// Zustandswechsel:

				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE || Button.ENTER.isDown()) 				// Abfrage ob Wechsel in Pause Modus
				{
					currentModus = CurrentModus.PAUSE;
					while (Button.ENTER.isDown())
					{
						Thread.sleep(1);
					}
				}

				else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown())	// Abfrage ob Wechsel in Disconnect Modus
				{
					currentModus = CurrentModus.DISCONNECT;
					while (Button.ESCAPE.isDown())
					{
						Thread.sleep(1);
					}
				}

				else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_NOW)		// Abfrage ob Wechsel in Park Now Modus
				{
					currentModus = CurrentModus.PARK_NOW;
				}

				else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS) 		// Abfrage ob Wechsel in Park This Modus
				{
					currentModus = CurrentModus.PARK_THIS;
				}

				break;
			/********************************************************************************************************************************************************/
			case PARK_NOW:// wenn Modus Park_Now

				if (lastModus != CurrentModus.PARK_NOW)		// Initialbefehle (nur bei erstmaligen Aufruf)
				{
					control.setCtrlMode(ControlMode.LINE_CTRL);
					navigation.setDetectionState(true);
					currentStatus = CurrentStatus.DRIVING;
				}

				lastModus = currentModus;

				if (pruefeAufLuecke(navigation.getPose().getX(), navigation.getPose().getY(), navigation, currentModus,	hmi))	// wenn Parkl�cke in der N�he zum Einparken wechseln
				{
					currentModus = CurrentModus.EINPARKEN;
				}


				//Zustandswechsel:

				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE || Button.ENTER.isDown())
				{
					currentModus = CurrentModus.PAUSE;
					while (Button.ENTER.isDown())
					{
						Thread.sleep(1);
					}
				}

				else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown())
				{
					currentModus = CurrentModus.DISCONNECT;
					while (Button.ESCAPE.isDown())
					{
						Thread.sleep(1);
					}
				}

				else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS)
				{
					currentModus = CurrentModus.PARK_THIS;
				}

				else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT)
				{
					currentModus = CurrentModus.SCOUT;
				}

				break;
			/********************************************************************************************************************************************************/
			case PARK_THIS:		// wenn Modus Park_This

				if (lastModus != CurrentModus.PARK_THIS)		// Initialbefehle (nur bei erstmaligen Aufruf)
				{
					control.setCtrlMode(ControlMode.LINE_CTRL);
					navigation.setDetectionState(true);
					currentStatus = CurrentStatus.DRIVING;
				}

				lastModus = currentModus;

				if (pruefeAufLuecke(navigation.getPose().getX(), navigation.getPose().getY(), navigation, currentModus,	hmi)) // wenn Parkl�cke in der N�he zum Einparken wechseln
				{
					currentModus = CurrentModus.EINPARKEN;
				}


				// Zustandswechsel
				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE || Button.ENTER.isDown())
				{
					currentModus = CurrentModus.PAUSE;
					while (Button.ENTER.isDown())
					{
						Thread.sleep(1);
					}
				}

				else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown())
				{
					currentModus = CurrentModus.DISCONNECT;
					while (Button.ESCAPE.isDown())
					{
						Thread.sleep(1);
					}
				}

				else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_NOW)
				{
					currentModus = CurrentModus.PARK_NOW;
				}

				else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT)
				{
					currentModus = CurrentModus.SCOUT;
				}

				break;
			/********************************************************************************************************************************************************/
			case EINPARKEN: 		// wenn Modus Einparken

				double  xz1, yz1;

				if (lastModus != CurrentModus.EINPARKEN) 		// Initialbefehle (nur bei erstmaligen Aufruf)
				{												// Zielkoordinaten setzen:
					xz1 = Zielkoordinaten[0];
					yz1 = Zielkoordinaten[1];
					double [] a=(Pfadgenerator(Startkoordinaten[0], Startkoordinaten[1], xz1, yz1));

					LCD.drawString("xz1 "+xz1, 0, 0);
					
					LCD.drawString("yz1 "+yz1, 0, 1);
					LCD.drawString("xs1 "+Startkoordinaten[0], 0, 2);
					LCD.drawString("ys1 "+Startkoordinaten[1], 0, 3);
					LCD.drawString("a0 "+a[0], 0, 4);
					LCD.drawString("a1 "+a[1], 0, 5);
					LCD.drawString("a2 "+a[2], 0, 6);
					LCD.drawString("a3 "+a[3], 0, 7);
					control.setPath( a, false, new Pose((float)Startkoordinaten[0], (float)Startkoordinaten[1],navigation.getPose().getHeading()),
							new Pose((float)xz1, (float)yz1, navigation.getPose().getHeading()), line); 	// Pfad setzen
					control.setCtrlMode(ControlMode.PARK_CTRL);								// Park Control anschalten, damit Pfad abgefahren wird
					navigation.setDetectionState(false);
					currentStatus = CurrentStatus.DRIVING;
				}

				lastModus = currentModus;

				// Zustandswechsel:

				if (control.getParkStatus()) 					// zur�ck zu PAUSE, wenn fertig mit Einparken
				{
					currentModus = CurrentModus.PAUSE;
				}

				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE || Button.ENTER.isDown())
				{
					currentModus = CurrentModus.PAUSE;
					while (Button.ENTER.isDown())
					{
						Thread.sleep(1);
					}
				}

				else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown())
				{
					currentModus = CurrentModus.DISCONNECT;
					while (Button.ESCAPE.isDown())
					{
						Thread.sleep(1);
					}
				}

				break;
			/********************************************************************************************************************************************************/
			case AUSPARKEN:			// wenn Modus Ausparken

				double xs, ys, xz, yz;

				if (lastModus != CurrentModus.AUSPARKEN) 	// Initialbefehle (nur bei erstmaligen Aufruf)
				{											// Startkoordinaten setzen

                    if(line==1)	 // bei rechter Parkl�cke Koordinaten vertauschen
                    {
                        xs = navigation.getPose().getY();
                        ys = navigation.getPose().getX();
                    }
                    else
                    {
                        xs = navigation.getPose().getX();
                        ys = navigation.getPose().getY();
                    }

					xz = Startkoordinaten[0];  // Zielkoordinaten = Startkoordinaten von Einparken (da jetzt r�ckw�rts ausgeparkt wird)
					yz = Startkoordinaten[1];
					double[] b=Pfadgenerator(xs, ys, xz, yz);
					LCD.drawString("xz "+xz, 0, 0);
					
					LCD.drawString("yz "+yz, 0, 1);
					LCD.drawString("xs "+xs, 0, 2);
					LCD.drawString("ys "+ys, 0, 3);
					LCD.drawString("a0 "+b[0], 0, 4);
					LCD.drawString("a1 "+b[1], 0, 5);
					LCD.drawString("a2 "+b[2], 0, 6);
					LCD.drawString("a3 "+b[3], 0, 7);
                    control.setPath(b, true, navigation.getPose(), new Pose((float)xz, (float)yz, navigation.getPose().getHeading()),line); 	// Pfad setzen

					control.setCtrlMode(ControlMode.PARK_CTRL);

					navigation.setDetectionState(false);
					currentStatus = CurrentStatus.DRIVING;
				}

				lastModus = currentModus;

				// Zustandswechsel:

				if (control.getParkStatus()) // zur�ck zu SCOUT, wenn fertig mit Ausparken
				{
					currentModus = CurrentModus.SCOUT;
				}

				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE || Button.ENTER.isDown())
				{
					currentModus = CurrentModus.PAUSE;
					while (Button.ENTER.isDown())
					{
						Thread.sleep(1);
					}
				}

				else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown())
				{
					currentModus = CurrentModus.DISCONNECT;
					while (Button.ESCAPE.isDown())
{
						Thread.sleep(1);
					}
				}

				break;
			/********************************************************************************************************************************************************/
			case PAUSE:		// wenn Modus Pause

				if (lastModus != CurrentModus.PAUSE) 				// Initialbefehle (nur bei erstmaligen Aufruf)
				{
					ParkingSlot[] Slots=navigation.getParkingSlots();
					 monitor.writeNavigationComment("Array startet hier ");
						for (int i=0;i<Slots.length;i++ ) {	 
					    monitor.writeNavigationComment("X_BACK "+Slots[i].getBackBoundaryPosition().x);
						monitor.writeNavigationComment("Y_BACK "+Slots[i].getBackBoundaryPosition().y);
					    monitor.writeNavigationComment("X_FRONT "+Slots[i].getFrontBoundaryPosition().x);
					    monitor.writeNavigationComment("Y_FRONT "+Slots[i].getFrontBoundaryPosition().y);
					    monitor.writeNavigationComment("Parken_ID "+Slots[i].getID());
					    monitor.writeNavigationComment("STATUS "+Slots[i].getStatus());
					    monitor.writeNavigationComment("index "+i);
					    monitor.writeNavigationComment( "grose des Arrays:"+Slots.length);
	    				}
					control.setCtrlMode(ControlMode.INACTIVE);
					navigation.setDetectionState(false);
					currentStatus = CurrentStatus.INACTIVE;
				}

				lastModus = currentModus;

				// auf Zustandswechsel pr�fen:

				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT || Button.ENTER.isDown())
				{
					if (control.getParkStatus()) 				// wenn Roboter in Parkl�cke zun�chst ausparken
						currentModus = CurrentModus.AUSPARKEN;

					else 										// sonst direkter Wechsel in Scout Modus
					{
						currentModus = CurrentModus.SCOUT;
					}

					while (Button.ENTER.isDown())
					{
						Thread.sleep(1);
					}
				}

				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_NOW && !control.getParkStatus()) 	// nur verf�gbar wenn sich der Roboter nicht in einer Parkl�cke befindet
				{
					currentModus = CurrentModus.PARK_NOW;
				}

				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS && !control.getParkStatus()) 	// nur verf�gbar wenn sich der Roboter nicht in einer Parkl�cke befindet
				{
					currentModus = CurrentModus.PARK_THIS;
				}

				else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT || Button.ESCAPE.isDown())
				{
					currentModus = CurrentModus.DISCONNECT;
					while (Button.ESCAPE.isDown())
					{
						Thread.sleep(1);
					}
				}

				break;
			/********************************************************************************************************************************************************/
			case DISCONNECT:		// wenn Modus Disconnect

				currentStatus = CurrentStatus.EXIT;
				control.setCtrlMode(ControlMode.INACTIVE);
				navigation.setDetectionState(false);

				hmi.disconnect();
				/**
				 * NOTE: RESERVED FOR FUTURE DEVELOPMENT (PLEASE DO NOT CHANGE)
				 * // monitor.sendOfflineLog();
				 */
				monitor.stopLogging();
				System.exit(0);				// Programm beenden

				break;
			/********************************************************************************************************************************************************/

			default:
				break;

			} // switch case Modus ende

			Thread.sleep(100);

		} // while(true) ende

	} // main ende

	/**
	 * returns the actual state of the main finite state machine as defined by
	 * the requirements
	 *
	 * @return actual state of the main finite state machine
	 */

	public static CurrentStatus getCurrentStatus() 	// liefert aktuellen Status f�r HMI
	{
		return GuidanceAT.currentStatus;
	}


	private static double[] Pfadgenerator(double xs, double ys, double xz, double yz) // Pfadgenerator, liefert Koeffizienten zur�ck
	{
		double[] werte = new double[4];	// Koeffizientenarray

										// zus�tzliche Variablen:
		double a, b, c, d;
		double x1, x2, x3, x4, x5, x6, x7, x8, x9, x10;
		double y1, y2;
		double x11, x12, x13, x14;
		double y11, y12;

		x1 = 1 / (xs);
		x2 = 1 / (xs * xs);
		x3 = 1 / (xs * xs * xs);
		x4 = 1 / (xz);
		x5 = 1 / (xz * xz);
		x6 = 1 / (xz * xz * xz);
		x7 = 2 / (3 * xs);
		x8 = 1 / (3 * xs * xs);
		x9 = 2 / (3 * xz);
		x10 = 1 / (3 * xz * xz);
		y1 = ys / (xs * xs * xs);
		y2 = yz / (xz * xz * xz);
		x11 = (x8 - x2) / (x7 - x1) - (x5 - x2) / (x4 - x1);
		x12 = (-x3) / (x7 - x1) - (x6 - x3) / (x4 - x1);
		x13 = (x10 - x2) / (x9 - x1) - (x5 - x2) / (x4 - x1);
		x14 = (-x3) / (x9 - x1) - (x6 - x3) / (x4 - x1);
		y11 = (-y1) / (x7 - x1) - (y2 - y1) / (x4 - x1);
		y12 = (-y1) / (x9 - x1) - (y2 - y1) / (x4 - x1);

		d = (y12 / x13 - y11 / x11) / (x14 / x13 - x12 / x11);
		c = y11 / x11 - d * x12 / x11;
		b = (y2 - y1) / (x4 - x1) - c * (x5 - x2) / (x4 - x1) - d * (x6 - x3) / (x4 - x1);
		a = y1 - b * x1 - c * x2 - d * x3;

		/**
		 * y(x) = a*x� + b*x� + c*x + d
		 */
						// Speichern zum Zur�ckliefern
		werte[0] = d;
		werte[1] = c;
		werte[2] = b;
		werte[3] = a;
						// zweites Mal speichern f�r sp�teres Ausparken (Vereinfachung, hier nicht genutzt)

		Koeffizienten[0] = werte[0];
		Koeffizienten[1] = werte[1];
		Koeffizienten[2] = werte[2];
		Koeffizienten[3] = werte[3];

		return werte; // Koeffizienten zur�ckliefern
	}

	private static boolean pruefeAufLuecke(float x, float y, INavigation navigation, CurrentModus modus, INxtHmi hmi) // Roboterkoordinaten werden �bergeben in m
	{

		boolean erg = false; 	// Initialisierung mit false: "keine passende Parkl�cke gefunden"

		float d = 0.10f; 		// 2*d = Abstandsintervall von Parkl�cke f�r Erkennung
		float a = 0.15f; 		// Abstand von Parkl�ckenrand (ca 5cm + halbe Roboterl�nge)
		float b = 0.25f; 		// Einparktiefe
		float dAlpha = 5;		// Winkelgenauigkeit f�r Winkelbedingung
		float winkel =  (float)(navigation.getPose().getHeading() / Math.PI * 180); // aktueller Winkel des Roboters in �
		while(winkel >= 360)	// Winkel gr��er 360� zur�cksetzen, damit Winkelbereich nur von 0� bis 360�
		{
			winkel-=360;
		}

		float xp_f, xp_b, yp_f, yp_b, lp_x, lp_y; 	// jeweilige Koordinaten bzw.
													// L�ngen der Parkl�cken

		if (navigation.getParkingSlots() != null) 	// wenn Parkl�cken vorhanden
		{
			if (modus == CurrentModus.PARK_NOW) 	// im Park Now Modus alle Parkl�cken aus Array pr�fen
				for (int i = 0; i < navigation.getParkingSlots().length; i++) 	// alle vorhandenen Parkl�cken durchlaufen
				{
					ParkingSlot slot = navigation.getParkingSlots()[i];
					if (slot.getStatus() == ParkingSlotStatus.GOOD) 		// nur "gute" Parkl�cken �berpr�fen
					{

						xp_f = slot.getFrontBoundaryPosition().x;			// Front X position
						xp_b = slot.getBackBoundaryPosition().x;			// Back  X position
						yp_f = slot.getFrontBoundaryPosition().y;			// Front Y position
						yp_b = slot.getBackBoundaryPosition().y;			// Back Y position

						lp_x = xp_f - xp_b;		// X L�nge
						lp_y = yp_f - yp_b;		// y L�nge

						if (lp_x < 0)			// nur positive L�ngen zulassen
							lp_x *= -1;
						if (lp_y < 0)			// nur positive L�ngen zulassen
							lp_y *= -1;

						if (y < 0.15 && x < 1.60 && yp_f < 0.15 && xp_b < 1.60 && ((winkel < dAlpha) || winkel > 360-dAlpha) && (winkel > -dAlpha)) // Roboter und Parkl�cke auf unterer Linie
						{
							if ((xp_b - x) < d && (xp_b - x) > -d)		// wenn X Abstand im Intervall von +- d
							{
								erg = true;
								line=0;												//Roboter auf erster Linie
								Startkoordinaten[0] = navigation.getPose().getX(); 	// wird f�r sp�teres Ausparken gesetzt
								Startkoordinaten[1] = navigation.getPose().getY(); 	// wird f�r sp�teres Ausparken gesetzt
								Zielkoordinaten[0] = xp_b + lp_x - a;
								Zielkoordinaten[1] = y - b;
							}
						}

						if (y > 0.15 && x < 1.60 && yp_f > 0.15 && xp_b < 1.60 && winkel < 180+dAlpha && winkel > 180-dAlpha) // Roboter und Parkl�cke auf oberer Linie
						{
							if ((x - xp_b) < d && (x - xp_b) > -d)
							{
								erg = true;
								line=4;
								Startkoordinaten[0] = navigation.getPose().getX();
								Startkoordinaten[1] = navigation.getPose().getY();
								Zielkoordinaten[0] = xp_b - lp_x + a;
								Zielkoordinaten[1] = y + b;
							}
						}

						if (x > 1.60 && xp_f > 1.60) // Roboter und Parkl�cke auf rechter Linie
						{
							if ((yp_b - y) < d && (yp_b - y) > -d && winkel < 90+dAlpha && winkel > 90-dAlpha)
                            {
								erg = true;
								line=1;
								Startkoordinaten[0] =navigation.getPose().getY();
								Startkoordinaten[1] = navigation.getPose().getX();
								Zielkoordinaten[0] = yp_b + lp_y - a;
								Zielkoordinaten[1] = x + b;
							}
						}
					}
				}

			if (modus == CurrentModus.PARK_THIS)		// im Park This Modus nur eine Parkl�cke pr�fen (vorgegebene)
			{
				ParkingSlot slot = navigation.getParkingSlots()[hmi.getSelectedParkingSlot()];

				xp_f = slot.getFrontBoundaryPosition().x;
				xp_b = slot.getBackBoundaryPosition().x;
				yp_f = slot.getFrontBoundaryPosition().y;
				yp_b = slot.getBackBoundaryPosition().y;

				lp_x = xp_f - xp_b;
				lp_y = yp_f - yp_b;
				if (lp_x < 0)
					lp_x *= -1;
				if (lp_y < 0)
					lp_y *= -1;

				if (y < 0.15 && x < 1.60 && yp_f < 0.15 && xp_b < 1.60 && ((winkel < dAlpha) || winkel > 360-dAlpha) && (winkel > -dAlpha)) // Roboter und Parkl�cke auf unterer Linie
				{
					if ((xp_b - x) < d && (xp_b - x) > -d)
					{
						erg = true;
						line=0;
						Startkoordinaten[0] =navigation.getPose().getX();
						Startkoordinaten[1] = navigation.getPose().getY();
						Zielkoordinaten[0] = xp_b + lp_x - a;
						Zielkoordinaten[1] = y - b;
					}
				}

				if (y > 0.15 && x < 1.60 && yp_f > 0.15 && xp_b < 1.60 && winkel < 180+dAlpha && winkel > 180-dAlpha) // Roboter und Parkl�cke auf oberer Linie
				{
					if ((x - xp_b) < d && (x - xp_b) > -d)
					{
						erg = true;
						line=4;
						Startkoordinaten[0] =navigation.getPose().getX();
						Startkoordinaten[1] = navigation.getPose().getY();
						Zielkoordinaten[0] = xp_b - lp_x + a;
						Zielkoordinaten[1] = y + b;
					}
				}

				if (x > 1.60 && xp_f > 1.60 && winkel < 90+dAlpha && winkel > 90-dAlpha) // Roboter und Parkl�cke auf rechter Linie
				{
					if ((yp_b - y) < d && (yp_b - y) > -d)
					{
						erg = true;
						line=1;
						Startkoordinaten[0] =navigation.getPose().getY();
						Startkoordinaten[1] = navigation.getPose().getX();
						Zielkoordinaten[0] = yp_b + lp_y - a;
						Zielkoordinaten[1] = x + b;
					}
				}

			}

		} else
			erg = false; // Fehler (navigation.getParkingSlots() = null)

		return erg;
	}

	/**
	 * plots the actual pose on the robots display
	 *
	 * @param navigation
	 *            reference to the navigation class for getting pose information
	 */

	protected static void showData(INavigation navigation, IPerception perception)  // LCD Display zeichnen
	{
		LCD.clear();

		LCD.drawString("X (in cm): " + (navigation.getPose().getX() * 100), 0, 0);
		LCD.drawString("Y (in cm): " + (navigation.getPose().getY() * 100), 0, 1);
		LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading() / Math.PI * 180), 0, 2);

		/**
		 * perception.showSensorData();
		 *
		 * if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ) {
		 * LCD.drawString("HMI Mode SCOUT", 0, 3); } else if ( hmi.getMode() ==
		 * parkingRobot.INxtHmi.Mode.PAUSE ) { LCD.drawString("HMI Mode PAUSE",
		 * 0, 3); } else { LCD.drawString("HMI Mode UNKNOWN", 0, 3); }
		 */
	}
}
