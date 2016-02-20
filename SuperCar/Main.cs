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
using MonoBrickFirmware.Movement;
using System.Threading;

namespace SmallRobots.SuperCar
{
	/// <summary>
	/// Main class.
	/// </summary>
	class MainClass
	{
		public static void Main (string[] args)
		{
			// Start the SuperCar execution
			SuperCar superCar = new SuperCar();
			superCar.Start ();
		}
	}
}


