using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using PI;

namespace PI.Motion
{
    public class SingleAxis : GCS2
    {
        public SingleAxis(int ControllerID, string Channel, int SlaveID = -1)
        {
            this.ControllerID = ControllerID;
            this.Channel = Channel;
            this.SlaveID = SlaveID;
        }

        public int ControllerID { get; private set; }
        public string Channel { get; private set; }
        public int SlaveID { get; private set; }

        public double UpperLimit = 0.00;
        public double LowerLimit = 0.00;

        public double PositionTolerance = 0.01;

        public bool Connected = false;
        public bool IsReady = false;
        public bool NegativeCoordinateSystem = false;

        private double velocity = 6.0;
        private double position = 0.00;
        private double target = 0.00;

        // PID tuning parameters, allows setting and saving to hardware
        private double p = 0.00;
        private double i = 0.00;
        private double d = 0.00;

        public double P
        {
            get
            {
                if (SlaveID.Equals(-1)) return double.PositiveInfinity;

                StringBuilder AxisName = new StringBuilder(2048);
                qSAI(SlaveID, AxisName, 2047);

                double[] pValue = new double[1];
                if (qSPA(SlaveID, AxisName.ToString(), new uint[] { 0x1 }, pValue, null, 0).Equals(0)) { p = double.PositiveInfinity; return p; }

                p = pValue[0];
                return p;
            }
            set
            {
                if (SlaveID.Equals(-1)) return;

                StringBuilder AxisName = new StringBuilder(2048);
                qSAI(SlaveID, AxisName, 2047);

                if (SPA(SlaveID, AxisName.ToString(), new uint[] { 0x1 }, new double[] { value }, null).Equals(0).Equals(0)) { p = double.PositiveInfinity; return; }
                p = value;
            }
        }
        public double I
        {
            get
            {
                if (SlaveID.Equals(-1)) return double.PositiveInfinity;

                StringBuilder AxisName = new StringBuilder(2048);
                qSAI(SlaveID, AxisName, 2047);

                double[] iValue = new double[1];
                if (qSPA(SlaveID, AxisName.ToString(), new uint[] { 0x2 }, iValue, null, 0).Equals(0)) { i = double.PositiveInfinity; return i; }

                i = iValue[0];
                return i;
            }
            set
            {
                if (SlaveID.Equals(-1)) return;

                StringBuilder AxisName = new StringBuilder(2048);
                qSAI(SlaveID, AxisName, 2047);

                if (SPA(SlaveID, AxisName.ToString(), new uint[] { 0x2 }, new double[] { value }, null).Equals(0).Equals(0)) { i = double.PositiveInfinity; return; }
                i = value;
            }
        }
        public double D
        {
            get
            {
                if (SlaveID.Equals(-1)) return double.PositiveInfinity;

                StringBuilder AxisName = new StringBuilder(2048);
                qSAI(SlaveID, AxisName, 2047);

                double[] dValue = new double[1];
                if (qSPA(SlaveID, AxisName.ToString(), new uint[] { 0x3 }, dValue, null, 0).Equals(0)) { d = double.PositiveInfinity; return d; }

                d = dValue[0];
                return d;
            }
            set
            {
                if (SlaveID.Equals(-1)) return;

                StringBuilder AxisName = new StringBuilder(2048);
                qSAI(SlaveID, AxisName, 2047);

                if (SPA(SlaveID, AxisName.ToString(), new uint[] { 0x3 }, new double[] { value }, null).Equals(0).Equals(0)) { d = double.PositiveInfinity; return; }
                d = value;
            }
        }
        public double Target
        {
            get
            {
                double[] Targ = new double[1];

                if (qMOV(ControllerID, Channel, Targ).Equals(0)) return double.PositiveInfinity;
                    
                target = Targ[0];
                if (NegativeCoordinateSystem) return target * -1.0;
                return target;
            }
        }
        public double Position
        {
            get
            {
                double[] Pos = new double[1];

                if (qPOS(ControllerID, Channel, Pos).Equals(0)) return double.PositiveInfinity;

                position = Pos[0];
                if (NegativeCoordinateSystem) return position * -1.0;
                return position;
            }
        }
        public double Velocity
        {
            get
            { return velocity; }
            set
            {
                if (VEL(ControllerID, Channel, new double[] { value }).Equals(0)) { return; }

                Thread.Sleep(50);

                double[] Vel = new double[1];

                if (qVEL(ControllerID, Channel, Vel).Equals(0)) return;

                velocity = Vel[0];
            }
        }
        public bool SaveChangesToHardware()
        {
            if (SlaveID.Equals(-1)) return false;
            int Success = 0;
            try
            {
                Success = GcsCommandset(SlaveID, "WPA 100");
            }
            catch { }

            return Success.Equals(1);
        }
        public bool TurnOffServo()
        {
            if (!Connected) return true;
            return (SVO(ControllerID, Channel, new int[] { 0 }).Equals(1));
        }
        public bool TurnOnServo()
        {
            return (SVO(ControllerID, Channel, new int[] { 1 }).Equals(1));
        }
        public void GetLimits()
        {
            if (!Connected) return;

            double[] lowerLimit = new double[1];
            double[] upperLimit = new double[1];

            if (!qTMN(ControllerID, Channel, lowerLimit).Equals(0))
            {
                LowerLimit = lowerLimit[0];
            }
            if (!qTMX(ControllerID, Channel, upperLimit).Equals(0))
            {
                UpperLimit = upperLimit[0];
            }
        }
        public bool Configure(double LowerLimit, double UpperLimit)
        {
            if (!Connected) return false;
            if (SVO(ControllerID, Channel, new int[] { 1 }).Equals(0)) return false; //Switch on Servo

            int[] Reference = new int[1];
            if (qFRF(ControllerID, Channel, Reference).Equals(0)) return false; // Determine reference status

            IsReady = Reference[0].Equals(1);

            this.UpperLimit = UpperLimit;
            this.LowerLimit = LowerLimit;

            return true;
        }
        public bool Configure()
        {
            if (!Connected) return false;

            if (SVO(ControllerID, Channel, new int[] { 1 }).Equals(0)) return false; //Switch on Servo
            int[] Reference = new int[1];
            if (qFRF(ControllerID, Channel, Reference).Equals(0)) return false; // Determine reference status

            IsReady = Reference[0].Equals(1);

            return true;
        }
        public string getError()
        {
            int Error = GetError(ControllerID);

            StringBuilder ErrorMessage = new StringBuilder(1024);
            TranslateError(Error, ErrorMessage, 1023);

            return ErrorMessage.ToString();
        }
        public bool isMoving(int controllerID, string channel)
        {
            if (!Connected) { return false; }

            Thread.Sleep(50);

            int[] ismoving = new int[] { 0 };
            if (IsMoving(controllerID, channel, ismoving).Equals(0)) return false;

            return (ismoving[0].Equals(1));
        }
        public bool Move(double Coordinate)
        {
            if (!Connected) return false;
            if ((Coordinate - PositionTolerance) > UpperLimit || (Coordinate + PositionTolerance) < LowerLimit) return false;

            int[] ServoOn = new int[] { 0 };
            if (qSVO(ControllerID, Channel, ServoOn).Equals(0)) return false; //Query Servo status, must be on
            if (ServoOn[0].Equals(0))
            {
                if (!Configure()) return false;
                if (qSVO(ControllerID, Channel, ServoOn).Equals(0)) return false;

                if (ServoOn[0].Equals(0)) return false;
            }

            if (MOV(ControllerID, Channel, new double[] { Coordinate }).Equals(0)) return false;

            return true;
        }
        private bool SetAxisReference(int controllerID, string channel)
        {
            int[] Reference = new int[1];
            if (qFRF(controllerID, channel, Reference).Equals(0)) return false; // Determine reference status

            if (Reference[0].Equals(0))
            {
                if (FRF(controllerID, channel).Equals(0)) return false; // Failed referencing 

                int ReferencingFinished = 0;
                while (ReferencingFinished.Equals(0))
                {
                    if (IsControllerReady(controllerID, ref ReferencingFinished).Equals(0)) return false;

                    Thread.Sleep(50);
                }
            }

            return true;
        }
        public bool SetAxisReference()
        {
            int[] Reference = new int[1];
            if (qFRF(ControllerID, Channel, Reference).Equals(0)) return false; // Determine reference status

            if (Reference[0].Equals(0))
            {
                if (FRF(ControllerID, Channel).Equals(0)) return false; // Failed referencing 

                int ReferencingFinished = 0;
                while (ReferencingFinished.Equals(0))
                {
                    if (IsControllerReady(ControllerID, ref ReferencingFinished).Equals(0)) return false;

                    Thread.Sleep(50);
                }
            }

            return true;
        }
    }
}
