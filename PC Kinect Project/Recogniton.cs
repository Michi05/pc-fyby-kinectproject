using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Research.Kinect.Nui;

namespace FallRecognition
{
    public static class Recogniton
    {

        /// <summary>
        /// Recognize current pose
        /// </summary>
        /// <param name="joints">Skelet positions</param>
        /// <param name="confidenceAngle">Maximum possible angle between left and right part body angles </param>
        /// <param name="standPoseFactor">Factor for recognition standing and sitting poses</param>
        /// <param name="isAutomaticChoiceAngle"> Chice flag of calculation angle between vectors HipShoulder and KneeShoulder
        /// if true nereast to sensor part body choiced else angle calculate as average between  left and right part body angles
        /// </param>
        /// <param name="angleShift">angle shift</param>
        /// <param name="recognitionHeadDistance">First time recognition distance of skelet head</param>
        /// <param name="leftAngle">left body part angle between vectors HipShoulder and KneeShoulder</param>
        /// <param name="rightAngle">left body part angle between vectors HipShoulder and KneeShoulder</param>
        /// <param name="currentHeadDistance">current Head Distance</param>
        /// <returns></returns>
        public static string RecognizePose(Microsoft.Research.Kinect.Nui.JointsCollection joints, double confidenceAngle, double standPoseFactor,
            bool isAutomaticChoiceAngle, double  angleShift,
            ref double recognitionHeadDistance, out double leftAngle, out double rightAngle, out double currentHeadDistance,
            out double bodyFloorAngle, out double hipsKneesHigh, out double headHigh)
        {

            string result;

            rightAngle = CalculateAngle(joints[JointID.ShoulderLeft], joints[JointID.HipRight], joints[JointID.KneeRight]);
            leftAngle = CalculateAngle(joints[JointID.ShoulderRight], joints[JointID.HipLeft], joints[JointID.KneeLeft]);

        // MICHI: New vble addings:
            List<Joint> highJoints = new List<Joint>();
            highJoints.Add(joints[JointID.Spine]);
            highJoints.Add(joints[JointID.HipCenter]);
            List<Joint> lowJoints = new List<Joint>();
            lowJoints.Add(joints[JointID.KneeLeft]);
            lowJoints.Add(joints[JointID.KneeRight]);
            bodyFloorAngle = CalculateAngle(highJoints, lowJoints);
//            bodyFloorAngle = RotateX((float)0.0, joints[JointID.Head]).Y;
        // If a knee is too high it can mean that the person is sitting or fallen or something else.
        //the more negative value is the most interesting since it doesn't mean anything if it is positive.
            hipsKneesHigh = joints[JointID.HipRight].Position.Y - joints[JointID.KneeRight].Position.Y;
            if (joints[JointID.HipLeft].Position.Y - joints[JointID.KneeLeft].Position.Y < hipsKneesHigh)
                hipsKneesHigh = joints[JointID.HipLeft].Position.Y - joints[JointID.KneeLeft].Position.Y;
            headHigh = joints[JointID.Head].Position.Y; // This value doesn't seem to be really realiable...

            if (recognitionHeadDistance == 0)
            {
                recognitionHeadDistance = vectorNorm(joints[JointID.Head].Position.X, joints[JointID.Head].Position.Y,
                    joints[JointID.Head].Position.Z);
            }

            currentHeadDistance = vectorNorm(joints[JointID.Head].Position.X, joints[JointID.Head].Position.Y,
                    joints[JointID.Head].Position.Z);
         

            // If first headDistance is greater or equal than the product between the current one and the standPoseFactor (1.1)
//            if (!(recognitionHeadDistance < standPoseFactor * currentHeadDistance))
            if (joints[JointID.HandLeft].Position.Y > joints[JointID.Head].Position.Y || joints[JointID.HandRight].Position.Y > joints[JointID.Head].Position.Y)
            { // Raising at least one hand // MICHI: I CHANGED this so it detects both-hand risings and not only the first checked.
                if (joints[JointID.HandLeft].Position.Y < joints[JointID.Head].Position.Y)
                    result = "Raising right hand";
                else if (joints[JointID.HandRight].Position.Y < joints[JointID.Head].Position.Y)
                    result = "Raising left hand";
                else
                    result = "Raising BOTH hands";
            }
            else
            {
                // If the angle is in the confidence interval
                if (Math.Abs(rightAngle - leftAngle) < confidenceAngle)
                {
                    double angle;
                    if (!isAutomaticChoiceAngle)
                        angle = (rightAngle + leftAngle) / 2;
                    else
                    {
                        // Get distance to both solders
                        double shLD = vectorNorm(joints[JointID.ShoulderLeft].Position.X, joints[JointID.ShoulderLeft].Position.Y,
                            joints[JointID.ShoulderLeft].Position.Z);
                        double shRD = vectorNorm(joints[JointID.ShoulderRight].Position.X, joints[JointID.ShoulderRight].Position.Y,
                           joints[JointID.ShoulderRight].Position.Z);
                        // The nearest shoulder's angle is the one taken into account
                        if (shLD > shRD)
                            angle = rightAngle;
                        else
                            angle = leftAngle;
                    }
                    result = "Sitting pose... somehow";
                }
                else
                    result = "Standing pose";
            }

            result = result + calculatePercentage(joints);

            return result;

        }

        private static string calculatePercentage(JointsCollection joints)
        {
            double percentage = 0;
            List<Joint> myJoints = new List<Joint>();
            myJoints.Add(joints[JointID.HipCenter]);
            myJoints.Add(joints[JointID.HipLeft]);
            myJoints.Add(joints[JointID.HipRight]);
            myJoints.Add(joints[JointID.ElbowLeft]);
            myJoints.Add(joints[JointID.ElbowRight]);
            myJoints.Add(joints[JointID.ShoulderCenter]);
            myJoints.Add(joints[JointID.ShoulderRight]);
            myJoints.Add(joints[JointID.ShoulderLeft]);
            myJoints.Add(joints[JointID.Spine]);
            myJoints.Add(joints[JointID.Head]);

            for (int i=0; i< myJoints.Count; i++)
                if (myJoints[i].Position.Y < -0.5)
                    percentage += ((double)100)/myJoints.Count;
            // MICHI: We need to add weights above (the head is not supposed to be in the floor but the hips may be)
            //and 2 levels of high.

            string result = "Falling probability = " + percentage.ToString() + "%.";
            return result;
        }

        /// <summary>
        /// Calculate angle between vectors HipShoulder and KneeShoulder
        /// </summary>
        /// <param name="shoulder">shoulder position</param>
        /// <param name="hip">hip position</param>
        /// <param name="knee">knee position</param>
        /// <returns></returns>
        public static double CalculateAngle(Joint shoulder, Joint hip, Joint knee)
        {
            double angle = 0;
            double shrhX = shoulder.Position.X - hip.Position.X;
            double shrhY = shoulder.Position.Y - hip.Position.Y;
            double shrhZ = shoulder.Position.Z - hip.Position.Z;
            double hsl = vectorNorm(shrhX, shrhY, shrhZ);
            double unrhX = knee.Position.X - hip.Position.X;
            double unrhY = knee.Position.Y - hip.Position.Y;
            double unrhZ = knee.Position.Z - hip.Position.Z;
            double hul = vectorNorm(unrhX, unrhY, unrhZ);
            double mhshu = shrhX * unrhX + shrhY * unrhY + shrhZ * unrhZ;

            double x = mhshu / (hul * hsl);
            if (x != Double.NaN)
            {
                if (-1 <= x && x <= 1)
                {
                    double angleRad = Math.Acos(x);
                    angle = angleRad * (180.0 / 3.1416);
                }
                else
                    angle = 0;
            }
            else
                angle = 0;
            return angle;
        }

        private static double CalculateAngle(List<Joint> highJoints, List<Joint> lowJoints)
        {
            Vector highVec = new Vector();
            Vector lowVec = new Vector();
            highVec.X = highVec.Y = highVec.Z = 0;
            for (int i = 0; i < highJoints.Count; i++)
            {
                highVec.X += highJoints[i].Position.X;
                highVec.Y += highJoints[i].Position.Y;
                highVec.Z += highJoints[i].Position.Z;
            }
            highVec.X /= highJoints.Count;
            highVec.Y /= highJoints.Count;
            highVec.Z /= highJoints.Count;

            lowVec.X = lowVec.Y = lowVec.Z = 0;
            for (int i = 0; i < lowJoints.Count; i++)
            {
                lowVec.X += lowJoints[i].Position.X;
                lowVec.Y += lowJoints[i].Position.Y;
                lowVec.Z += lowJoints[i].Position.Z;
            }
            lowVec.X /= lowJoints.Count;
            lowVec.Y /= lowJoints.Count;
            lowVec.Z /= lowJoints.Count;

//            return vectorNorm(highVec.X - lowVec.X, highVec.Y - lowVec.Y, highVec.Z - lowVec.Z);
            return (highVec.Y - lowVec.Y);
        }



        // MICHI: New method in case we use the body-floor angle
        /// <summary>
        /// Calculate angle between high-low vector and the floor
        /// PRECONDITION: The camera is supposed to be aligned with the floor, not side tilted (it may be tilted to the front or back)
        /// </summary>
        /// <param name="shoulder">high reference point</param>
        /// <param name="hip">low reference point</param>
        /// <returns></returns>
        public static double CalculateAngle(Vector high, Vector low)
        {
            double angle = 0;
            double vecX = high.X - low.X;
            double vecY = high.Y - low.Y;
            double vecZ = high.Z - low.Z;
            double hsl = vectorNorm(vecX, vecY, vecZ);
            // The floor vector for this version is the vertical proyection of the body on the floor so it should be the minimized distance
            double floorX = vecX;
            double floorY = 0;
            double floorZ = vecZ;
            double hul = vectorNorm(floorX, floorY, floorZ);
            double mhshu = vecX * floorX + vecY * floorY + vecZ * floorZ;

            double x = mhshu / (hul * hsl);
            if (x != Double.NaN)
            {
                if (-1 <= x && x <= 1)
                {
                    double angleRad = Math.Acos(x);
                    angle = angleRad * (180.0 / 3.1416);
                }
                else
                    angle = 0;
            }
            else
                angle = 0;
            return angle;
        }

        /// <summary>
        /// Euclidean norm of 3-component Vector
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <returns></returns>
        public static double vectorNorm(double x, double y, double z)
        {
            return Math.Sqrt(Math.Pow(x, 2) + Math.Pow(y, 2) + Math.Pow(z, 2));
        }

        // TODO: Recheck the following region:

        #region Rotation

        public static Vector RotateX(float angle, Joint joint)
        {
            float radAngle = (float)((angle * Math.PI) / 180.0);
            Vector iniCoords = new Vector();
            Vector newCoords = new Vector();
            iniCoords.X = joint.Position.X;
            iniCoords.Y = joint.Position.Y;
            iniCoords.Z = joint.Position.Z;

            // MICHI: BEWARE OF THIS, the double-float conversion is not really checked and there could be missing information... unimportant?
            newCoords.X = joint.Position.X;
            newCoords.Y = (float)(iniCoords.Y * Math.Cos(radAngle) + iniCoords.Z * Math.Sin(radAngle));
            newCoords.Z = (float)(iniCoords.Z * Math.Cos(radAngle) - iniCoords.Y * Math.Sin(radAngle));
            
            return newCoords;
        }

        #endregion Rotation
    }
}
