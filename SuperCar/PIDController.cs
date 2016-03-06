////////////////////////////////////////////////////////////////////////////////// 
// PID Algorithm                                                                //
//                                                                              //
// Copyright 2015                                                               //
// SmallRobots.it                                                               //
//                                                                              //
// This code is happily shared under                                            //
// The Code Project Open License (CPOL) 1.02                                    //
//                                                                              //
// THIS WORK IS PROVIDED "AS IS", "WHERE IS" AND "AS AVAILABLE",                //
// WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES OR CONDITIONS OR GUARANTEES.       //
// YOU, THE USER, ASSUME ALL RISK IN ITS USE, INCLUDING COPYRIGHT INFRINGEMENT, //
// PATENT INFRINGEMENT, SUITABILITY, ETC. AUTHOR EXPRESSLY DISCLAIMS ALL        //
// EXPRESS, IMPLIED OR STATUTORY WARRANTIES OR CONDITIONS, INCLUDING WITHOUT    // 
// LIMITATION, WARRANTIES OR CONDITIONS OF MERCHANTABILITY, MERCHANTABLE        //
// QUALITY OR FITNESS FOR A PARTICULAR PURPOSE, OR ANY WARRANTY OF TITLE        //
// OR NON-INFRINGEMENT, OR THAT THE WORK (OR ANY PORTION THEREOF) IS CORRECT,   //
// USEFUL, BUG-FREE OR FREE OF VIRUSES. YOU MUST PASS THIS DISCLAIMER ON        //
// WHENEVER YOU DISTRIBUTE THE WORK OR DERIVATIVE WORKS                         //
//////////////////////////////////////////////////////////////////////////////////

using System;
using MonoBrickFirmware;
using MonoBrickFirmware.Movement;
using System.Threading;

namespace SmallRobots.Controllers
{
	/// <summary>
	/// EV3 Medium motor position PID
	/// </summary>
	public class MediumMotorPositionPID : PIDController
	{
		#region Properties
		/// <summary>
		/// Gets or sets the Motor to be used for regulation
		/// </summary>
		/// <value>The motor.</value>
		public Motor Motor { get; set; }

		/// <summary>
		/// Get the output signal of the PID controller
		/// </summary>
		/// <value>The output.</value>
		public override sbyte OutputSignal 
		{
			get {
				return outputSignal;
			}
			protected set {
				if (outputSignal != value) {
					outputSignal = value;
					Motor.SetPower (outputSignal);
				}
			}
		}
		#endregion

		#region Constructors
		/// <summary>
		/// Initializes a new instance of the <see cref="Natale.SuperPid.MediumMotorPositionPID"/> class.
		/// </summary>
		public MediumMotorPositionPID() : base()
		{
			// Fields initialization
			init();
		}

		/// <summary>
		/// Fields initialization
		/// </summary>
		private void init()
		{
			// Default constants
			Kp = 5f;
			Ki = 0.5f;
			Kd = 0f;
			SampleTime = 10;
		}
		#endregion

		#region Public Methods
		/// <summary>
		/// Start the PID
		/// </summary>
		/// <exception cref="InvalidOperationException">Motor must be assigned before operating the PID</exception>
		public override void Start()
		{
			// Check for motor validity
			if (Motor == null) {
				InvalidOperationException ex = new InvalidOperationException ("Motor must be assigned before operating the PID");
				throw (ex);
			}

			// Stop motor
			Motor.SetSpeed(0);
			Motor.Brake ();
			Motor.ResetTacho();

			// Start the PID
			base.Start ();
		}

		/// <summary>
		/// Stop the PID
		/// </summary>
		/// <exception cref="InvalidOperationException">Motor must be assigned before operating the PID</exception>
		public override void Stop()
		{
			// Stop the PID
			base.Stop();

			// Check for motor validity
			if (Motor == null) {
				InvalidOperationException ex = new InvalidOperationException ("Motor must be assigned before operating the PID");
				throw (ex);
			}
		
			// Stop motor
			Motor.SetSpeed(0);
			Motor.Brake ();

		}
		#endregion
	}

	/// <summary>
	/// PID Controller class
	/// </summary>
	public class PIDController
	{
		#region Properties
		/// <summary>
		/// Get or sets the input used for PID calculation
		/// </summary>
		/// <value>The input.</value>
		public sbyte InputSignal { get; set; }

		protected sbyte outputSignal;
		/// <summary>
		/// Get the the control signal of the PID controller
		/// </summary>
		/// <value>The output.</value>
		public virtual sbyte OutputSignal
		{
			get {
				return outputSignal;
			}
			protected set {
				if (outputSignal != value) {
					outputSignal = value;
				}
			}
		}

		/// <summary>
		/// Get or sets the SetPoint used for regulation
		/// </summary>
		public sbyte SetPoint { get; set;}

		private int sampleTime;
		/// <summary>
		/// Get or sets the sample time
		/// </summary>
		/// <value>The sample time.</value>
		/// <exception cref="ArgumentException">Thrown if value is non positive</exception>
		public int SampleTime
		{
			get {
				return sampleTime;
			}
			set{ 
				if (value < 0) {
					ArgumentException ex = new ArgumentException ("The sample time must be a positive number");
					throw(ex);
				} else if (sampleTime != value) {
					sampleTime = value;
				}
			}
		}

		/// <summary>
		/// Get or sets the minimum power
		/// or in other words the minimum allowable
		/// value for the control signal
		/// </summary>
		/// <value>The minimum power.</value>
		public sbyte MinPower { set; get; }

		/// <summary>
		/// Get or sets the maximum power
		/// or in other words the maximum allowable
		/// value for the control signal
		/// </summary>
		/// <value>The max power.</value>
		public sbyte MaxPower { get; set;}


		private float kp;
		/// <summary>
		/// Get or sets the proportional constant
		/// </summary>
		/// <value>The Kp</value>
		/// <exception cref="ArgumentException">Thrown if value is non positive</exception>
		public float Kp {
			get {
				return kp;
			}
			set {
				if (value < 0) {
					ArgumentException ex = new ArgumentException ("The proportional constant must be a positive number");
					throw(ex);
				} else if (kp != value) {
					kp = value;
					UpdateKs();
				}
			}
		}

		private float ki;
		/// <summary>
		/// Get or sets the integral constant
		/// </summary>
		/// <value>The Ki</value>
		/// <exception cref="ArgumentException">Thrown if value is non positive</exception>
		public float Ki { 
			get {
				return ki;
			}
			set {
				if (value < 0) {
					ArgumentException ex = new ArgumentException ("The integral constant must be a positive number");
					throw(ex);
				} else if (ki != value) {
					ki = value;
					UpdateKs ();
				}
			}
		}

		private float kd;
		/// <summary>
		/// Get or sets the derivative constant
		/// </summary>
		/// <value>The Kd</value>
		/// <exception cref="ArgumentException">Thrown if value is non positive</exception>
		public float Kd {
			get {
				return kd;
			}
			set {
				if (value < 0) {
					ArgumentException ex = new ArgumentException ("The derivative constant must be a positive number");
					throw(ex);
				} else if (kd != value) {
					kd = value;
					UpdateKs ();
				}
			}
		}
		#endregion

		#region Fields
		/// <summary>
		/// Constants used int the "speed form" of the discrete PID algorithm
		/// </summary>
		private float k1;
		private float k2;
		private float k3;

		// PIDThread
		private EventWaitHandle stopPIDThread; 
		private Thread PIDThread;
		#endregion

		#region Constructors
		/// <summary>
		/// Default constructor
		/// </summary>
		public PIDController ()
		{
			// Fields initialization
			init();
		}

		/// <summary>
		/// Fields initialization
		/// </summary>
		private void init()
		{
			Ki = 0;
			Kp = 0;
			Kd = 0;
			sampleTime = 10;

			// PID Thread
			stopPIDThread = new ManualResetEvent(false);
			PIDThread = new Thread (PIDThreadComputation);
		}
		#endregion

		#region Public methods
		/// <summary>
		/// Starts the PID
		/// </summary>
		public virtual void Start()
		{
			PIDThread.Start();
		}

		/// <summary>
		/// Stops the PID
		/// </summary>
		public virtual void Stop()
		{
			stopPIDThread.Set ();
			if (PIDThread.IsAlive) {
				PIDThread.Join ();
			}
		}
		#endregion

		#region Private methods
		/// <summary>
		/// Updates the K1, K2 and K3 constants
		/// </summary>
		private void UpdateKs(){
			k1 = ki + kp + kd;
			k2 = -kp - 2 * kd;
			k3 = kd;
		}

		/// <summary>
		/// The core of the PID alogrithm computation
		/// </summary>
		protected void PIDThreadComputation(){
			Thread.CurrentThread.IsBackground = true;

			float e;		// error at instant k
			float e1;		// error at instant k-1
			float e2;		// error at instant k-1
			float u;		// control signal at instant k
			float delta_u;	// control signal increment at instant k

			// Initialization
			e = 0f;
			e1 = 0f;
			e2 = 0f;
			u = 0f;
			delta_u = 0f;

			// PID Computation
			while (!stopPIDThread.WaitOne(SampleTime)) {
				// updating "memory" variables
				e2 = e1;
				e1 = e;

				// updating error
				e = SetPoint - InputSignal;

				// computating delta_u and u
				delta_u = k1 * e + k2 * e1 + k3 * e2;
				u = u + delta_u;

				// Saturatin u
				if (u > MaxPower)
					u = MaxPower;
				if (u < MinPower)
					u = MinPower;
				
				OutputSignal = (sbyte) u;
			}
		}
		#endregion
	}
}

