//////////////////////////// 
// Super Car              //
// Trackfollowing program //
//                        // 
// Copyright 2015         //  
// Oreste Riccardo Natale //
//////////////////////////// 

// Project inclusions
using System;
using MonoBrickFirmware;
using MonoBrickFirmware.Display.Dialogs;
using MonoBrickFirmware.Display;
using MonoBrickFirmware.UserInput;
using MonoBrickFirmware.Movement;
using MonoBrickFirmware.Sensors;
using System.Threading;
using SmallRobots.Controllers;

namespace SmallRobots.SuperCar
{
	public class SuperCar
	{
		#region Field
		/// <summary>
		/// Reflected light sensor
		/// </summary>
		private EV3ColorSensor colorSensor;
		private int reflectedLight;

		// Right wheel motor
		private Motor rightEngine;

		// Left wheel motor
		private Motor leftEngine;

		// Steering wheel motor
		private Motor steerWheel;
		private int gearRatio;
		private sbyte steerPower;
		private MediumMotorPositionPID steerPID;

		// Sensor UpdateThread
		private EventWaitHandle stopSensorUpdate; 
		private int sensorUpdateSamplingTime; 
		private Thread sensorUpdateThread;

		// Drive Thread
		private EventWaitHandle stopDrive;
		private int driveSamplingTime;
		private Thread driveThread;

		// Graphic LCDThread
		private EventWaitHandle stopLCDThread;
		private int lCDThreadSampleTime;
		private Thread lCDThread;

		// Current steering angle
		private int currentSteeringAngle;
		#endregion

		#region Costruttori
		/// <summary>
		/// Default constructor
		/// </summary>
		public SuperCar ()
		{
			// Field initialization
			init();
		}

		/// <summary>
		/// // Field initialization
		/// </summary>
		private void init()
		{
			// Connected sensors
			colorSensor = new EV3ColorSensor (SensorPort.In1, ColorMode.Reflection);

			// Motors
			steerWheel = new Motor (MotorPort.OutA);
			rightEngine = new Motor(MotorPort.OutB);
			leftEngine = new Motor (MotorPort.OutC);
			rightEngine.Off ();
			leftEngine.Off ();
			steerWheel.Off ();
			steerWheel.ResetTacho ();

			// SteeringWheel motor parameters
			gearRatio = 3;
			steerPower = 30;
			currentSteeringAngle = 0;

			// PositionPID
			steerPID = new MediumMotorPositionPID();
			steerPID.Motor = steerWheel;
			steerPID.MaxPower = steerPower;
			steerPID.MinPower = (sbyte) - steerPower;

			// Sensors update thread
			stopSensorUpdate = new ManualResetEvent(false);
			sensorUpdateSamplingTime = 50;
			sensorUpdateThread = new Thread (SensorUpdateThread);

			// Drive thread
			stopDrive = new ManualResetEvent(false);
			driveSamplingTime = 100;
			driveThread = new Thread (DriveThread);

			stopLCDThread = new ManualResetEvent (false);
			lCDThreadSampleTime = 250;
			lCDThread = new Thread (LCDThread);
		}
		#endregion

		#region Thread
		/// <summary>
		/// LCD Thread
		/// </summary>
		private void LCDThread()
		{
			Thread.CurrentThread.IsBackground = true;
			Font f = Font.MediumFont;
			Point offset = new Point(0,25);
			Point p = new Point(10, Lcd.Height-75);
			Point boxSize = new Point(100, 24);
			Rectangle box = new Rectangle(p, p+boxSize);

			while (!stopLCDThread.WaitOne(lCDThreadSampleTime))
			{
				Lcd.Clear (); 
				Lcd.WriteTextBox (f, box , "Light = " + reflectedLight, true); 
				Lcd.WriteTextBox (f, box + offset , "Steer " + currentSteeringAngle, false);  
				Lcd.Update (); 
			}
		}

		/// <summary>
		/// Drive thread
		/// </summary>
		private void DriveThread()
		{
			Thread.CurrentThread.IsBackground = true;

			// Parameters
			sbyte setPoint = 15;
			sbyte maxDriveSpeed = 30;
			sbyte driveSpeed = 0;
			sbyte steeringOffset = 0;

			// PID Variables
			sbyte deadband = 5;
			sbyte maxOutput = 40;
			sbyte error = 0;
			sbyte u = 0;
			float kp = 0.8f;

			// Electronic differential variables;
			sbyte leftSpeed = driveSpeed;
			sbyte rightSpeed = driveSpeed;

			while (!stopDrive.WaitOne (driveSamplingTime)) 
			{
				// Compute tracking error
				error = (sbyte) (setPoint - reflectedLight);

				// Determine if error is within deadband and if not, compute the correction
				if (Math.Abs (error) > deadband)
					u = (sbyte)(-kp * error);
				else
					u = 0;

				// Determine speed
				if (Math.Abs (error) <= deadband)
					driveSpeed = maxDriveSpeed;
				else
					if ((Math.Abs (error) > deadband) || (Math.Abs (error) < 3 * deadband))
						driveSpeed = (sbyte) (maxDriveSpeed / 2);
					else if (Math.Abs (error) >= 3*deadband)
						driveSpeed = (sbyte) (maxDriveSpeed / 3);

				// See if the correction is outside admitted bounds
				if (u > maxOutput)
					u = maxOutput;
				if (u < (sbyte) -maxOutput)
					u = (sbyte) -maxOutput;

				steeringOffset = 0;

				// Adjust rear wheels speed when steering
				if (u < 0) {
					// Steering to right
					leftSpeed = driveSpeed;
					rightSpeed = (sbyte) (driveSpeed * Math.Abs (maxOutput + u) / maxOutput);

					steeringOffset = -20;

				} else if (u > 0) {
					// Steering to left
					leftSpeed = (sbyte) (driveSpeed * Math.Abs (maxOutput - u) / maxOutput);
					rightSpeed = driveSpeed;

				}

				steerPID.SetPoint = (sbyte) (-gearRatio * (u + steeringOffset));

				rightEngine.SetSpeed ((sbyte) -rightSpeed);
				leftEngine.SetSpeed ((sbyte) -leftSpeed);

				Buttons.LedPattern (1);
			}
			Buttons.LedPattern (0);

		}


		// Led Patterns
		// 0 - Spento
		// 1 - Verde orizzontale
		// 2 - Rosso orizzontale
		// 3 - Arancio orizzontale
		// 4 - Verde orizzontale
		// 5 - Rosso orizzontale

		/// <summary>
		/// Sensors update thread
		/// </summary>
		private void SensorUpdateThread()
		{
			Thread.CurrentThread.IsBackground = true;

			while (!stopSensorUpdate.WaitOne (sensorUpdateSamplingTime)) 
			{
				// Ev3 Color sensor in reflected light mode
				reflectedLight = colorSensor.Read ();

				// Current steering angle and asynchronous feed to the Steering Thread
				currentSteeringAngle = steerWheel.GetTachoCount ();
				steerPID.InputSignal = (sbyte) currentSteeringAngle;
			}
		}
		#endregion

		#region Metodi pubblici
		/// <summary>
		/// Starts the SuperCar
		/// </summary>
		public void Start()
		{
			// Program stop event
			ManualResetEvent terminateProgram = new ManualResetEvent(false);

			// Welcome messages
			LcdConsole.WriteLine ("SuperCar running");
			LcdConsole.WriteLine ("Enter to start");
			LcdConsole.WriteLine ("Left to steer left");
			LcdConsole.WriteLine ("Right to steer right");
			LcdConsole.WriteLine ("Esc to terminate");

			// Button events
			ButtonEvents buts = new ButtonEvents ();

			// Enter button
			buts.EnterPressed += () => 
			{  
				LcdConsole.WriteLine("Application Started");	
				sensorUpdateThread.Start();
				steerPID.Start();
				driveThread.Start();
				lCDThread.Start();
			}; 

			// Right button
			buts.RightPressed += () => 
			{
				
			};

			// Left button
			buts.LeftPressed += () => 
			{
				
			};

			// Escape button
			buts.EscapePressed += () => 
			{  
				Lcd.Clear();
				LcdConsole.Clear();
				LcdConsole.WriteLine("Application Terminating...");

				// Termina il thread di aggiornamento dei sensori
				stopSensorUpdate.Set();
				if (sensorUpdateThread.IsAlive)
				{
					// Ne attende la terminazione se è vivo
					sensorUpdateThread.Join();
				}
				LcdConsole.WriteLine("Sensors update thread terminated.");

				// Termina il thread di sterzata
				steerPID.Stop();
				LcdConsole.WriteLine("Steer thread terminated.");

				// Termina il thread di guida
				stopDrive.Set();
				if (driveThread.IsAlive)
				{
					// Ne attende la terminazione se è vivo
					driveThread.Join();
				}
				LcdConsole.WriteLine("Drive thread thread terminated.");

				// Termina il Thread di update del LCD
				stopLCDThread.Set();
				if (lCDThread.IsAlive)
				{
					// Ne attende la terminazione
					lCDThread.Join();
				}
				LcdConsole.WriteLine("LCD thread thread terminated.");
				LcdConsole.WriteLine("Application terminated.");

				// Spegne tutti i motori
				leftEngine.Off();
				rightEngine.Off();
				steerWheel.Off();

				// Aspetta un pochino
				Thread.Sleep(1000);

				// E termina
				terminateProgram.Set(); 
			}; 

			terminateProgram.WaitOne ();

		}
			
		#endregion
	}
}

