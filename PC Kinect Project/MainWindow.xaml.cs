/////////////////////////////////////////////////////////////////////////
//
//
// SittingPose software for detecting postures with Kinect
// Modified for better understanding and study in the LTU
// by Moulaye Ndiaye, Miguel García and Abdul Quyum
// No license. Classified document
//
/////////////////////////////////////////////////////////////////////////
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.ComponentModel;
using Microsoft.Research.Kinect.Nui;
//using Coding4Fun.Kinect.Wpf; 
using System.IO;

namespace FallRecognition
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region ctor & Window events
        public MainWindow()
        {
            InitializeComponent();
        }

        // Public vble initializations for MainWindow
        BackgroundWorker _worker = new BackgroundWorker();
        Runtime nui; //Kinect Runtime
        PlanarImage colorImage = new PlanarImage();
        ImageFrame depthImageFrame = new ImageFrame();
        byte[] ColoredBytes;

        double recognitionHeadDistance = 0;
        DateTime lastTime = DateTime.MaxValue;
        DateTime logTimeChecker = DateTime.MaxValue;
        double KinectAngle = 0;

        // We want to control how depth data gets converted into false-color data
        // for more intuitive visualization, so we keep 32-bit color frame buffer versions of
        // these, to be updated whenever we receive and process a 16-bit frame.
        const int RED_IDX = 2;
        const int GREEN_IDX = 1;
        const int BLUE_IDX = 0;

        // Every joint is assigned a color forming a "joint-colour" dictionary
        Dictionary<JointID,Brush> jointColors = new Dictionary<JointID,Brush>() { 
            {JointID.HipCenter, new SolidColorBrush(Color.FromRgb(169, 176, 155))},
            {JointID.Spine, new SolidColorBrush(Color.FromRgb(169, 176, 155))},
            {JointID.ShoulderCenter, new SolidColorBrush(Color.FromRgb(168, 230, 29))},
            {JointID.Head, new SolidColorBrush(Color.FromRgb(200, 0,   0))},
            {JointID.ShoulderLeft, new SolidColorBrush(Color.FromRgb(79,  84,  33))},
            {JointID.ElbowLeft, new SolidColorBrush(Color.FromRgb(84,  33,  42))},
            {JointID.WristLeft, new SolidColorBrush(Color.FromRgb(255, 126, 0))},
            {JointID.HandLeft, new SolidColorBrush(Color.FromRgb(215,  86, 0))},
            {JointID.ShoulderRight, new SolidColorBrush(Color.FromRgb(33,  79,  84))},
            {JointID.ElbowRight, new SolidColorBrush(Color.FromRgb(33,  33,  84))},
            {JointID.WristRight, new SolidColorBrush(Color.FromRgb(77,  109, 243))},
            {JointID.HandRight, new SolidColorBrush(Color.FromRgb(37,   69, 243))},
            {JointID.HipLeft, new SolidColorBrush(Color.FromRgb(77,  109, 243))},
            {JointID.KneeLeft, new SolidColorBrush(Color.FromRgb(69,  33,  84))},
            {JointID.AnkleLeft, new SolidColorBrush(Color.FromRgb(229, 170, 122))},
            {JointID.FootLeft, new SolidColorBrush(Color.FromRgb(255, 126, 0))},
            {JointID.HipRight, new SolidColorBrush(Color.FromRgb(181, 165, 213))},
            {JointID.KneeRight, new SolidColorBrush(Color.FromRgb(71, 222,  76))},
            {JointID.AnkleRight, new SolidColorBrush(Color.FromRgb(245, 228, 156))},
            {JointID.FootRight, new SolidColorBrush(Color.FromRgb(77,  109, 243))}
        };

        private void Window_Loaded(object sender, EventArgs e)
        {
            SetupKinect();
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            // Cleanup // MICHI: this is important, watch out
            nui.Uninitialize();
            Environment.Exit(0);
        }

        #endregion ctor & Window events


        #region Kinect setup

        private void enableSmoothness()
        {
            nui.SkeletonEngine.TransformSmooth = true;

            //MICHI: make sure to fine tune this
            //Use to transform and reduce jitter
            var parameters = new TransformSmoothParameters
            {
                Smoothing = 0.75f, //0.5f, //0.75f
                Correction = 0.0f, //0.5f, //0.0f
                Prediction = 0.05f, //0.0f
                JitterRadius = 0.70f, //0.05f
                MaxDeviationRadius = 0.1f //0.05f //0.04f
            };

            nui.SkeletonEngine.SmoothParameters = parameters;
        }

        private void SetupKinect()
        {
            if (Runtime.Kinects.Count < 1)
            {
                this.Title = "No Kinect connected";
                return;
            }
            else
            { // MICHI: If there are Kinects, the setup begins

                // Use first Kinect
                nui = Runtime.Kinects[0];
                CompositionTarget.Rendering += new EventHandler(CompositionTarget_Rendering);

                try
                {
                    // Initialize to do skeletal tracking
                    nui.Initialize(RuntimeOptions.UseDepthAndPlayerIndex | RuntimeOptions.UseSkeletalTracking | RuntimeOptions.UseColor);
                        //nui.Initialize(RuntimeOptions.UseSkeletalTracking);
                    enableSmoothness();
                }
                catch (InvalidOperationException)
                {
                    System.Windows.MessageBox.Show("Runtime initialization failed. Please make sure Kinect device is plugged in.");
                    return;
                }

                try
                {
                    nui.VideoStream.Open(ImageStreamType.Video, 2, ImageResolution.Resolution640x480, ImageType.Color);
                //DepthAndPlayerIndex ImageType
                    nui.DepthStream.Open(ImageStreamType.Depth, 2, ImageResolution.Resolution320x240, ImageType.DepthAndPlayerIndex);
                }
                catch (InvalidOperationException)
                {
                    System.Windows.MessageBox.Show("Failed to open stream. Please make sure to specify a supported image type and resolution.");
                    return;
                }

                // Register for events
                // Add event to receive skeletonCanvas data and Rendering event
                nui.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(nui_SkeletonFrameReady);
                nui.VideoFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_ColorFrameReady);
                // _worker.DoWork += new DoWorkEventHandler(_worker_DoWork);
                nui.DepthFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_DepthFrameReady);

                logTimeChecker = lastTime = DateTime.Now;
                LogMessageToFile(System.String.Format(
                        "\r\n\t{0:G}: PC Kinect Project application has been launched.", System.DateTime.Now));


            }
        }


        void CompositionTarget_Rendering(object sender, EventArgs e)
        {
            /*
            if (!_worker.IsBusy)
            {
                _worker.RunWorkerAsync();
            }
             * */
        }

        void _worker_DoWork(object sender, DoWorkEventArgs e)
        {
            Dispatcher.BeginInvoke((Action)delegate
            {

            });
        }

        #endregion Kinect setup



        #region depth

        void nui_DepthFrameReady(object sender, ImageFrameReadyEventArgs e)
        {
            //Convert depth information for a pixel into color information
            ColoredBytes = GenerateMonocoloredBytes(e.ImageFrame);

            depthImageFrame = e.ImageFrame;

            // This is the layout image for depth, I don't need it
            //            image1.Source = BitmapSource.Create(image.Width, image.Height, 96, 96, PixelFormats.Bgr32, null,
            //                ColoredBytes, image.Width * PixelFormats.Bgr32.BitsPerPixel / 8);
        }

        private byte[] GenerateColoredBytes(ImageFrame imageFrame)
        {
            int height = imageFrame.Image.Height;
            int width = imageFrame.Image.Width;

            //Depth data for each pixel
            Byte[] depthData = imageFrame.Image.Bits;


            //colorFrame contains color information for all pixels in image
            //Height x Width x 4 (Red, Green, Blue, empty byte)
            Byte[] colorFrame = new byte[imageFrame.Image.Height * imageFrame.Image.Width * 4];

            //Bgr32  - Blue, Green, Red, empty byte
            //Bgra32 - Blue, Green, Red, transparency 
            //You must set transparency for Bgra as .NET defaults a byte to 0 = fully transparent

            //hardcoded locations to Blue, Green, Red (BGR) index positions       
            const int BlueIndex = 0;
            const int GreenIndex = 1;
            const int RedIndex = 2;


            var depthIndex = 0;
            for (var y = 0; y < height; y++)
            {

                var heightOffset = y * width;

                for (var x = 0; x < width; x++)
                {

                    var index = ((width - x - 1) + heightOffset) * 4;

                    //var distance = GetDistance(depthData[depthIndex], depthData[depthIndex + 1]);
                    var distance = GetDistanceWithPlayerIndex(depthData[depthIndex], depthData[depthIndex + 1]);

                    if (distance <= 900)
                    {
                        //we are very close
                        colorFrame[index + BlueIndex] = 255;
                        colorFrame[index + GreenIndex] = 0;
                        colorFrame[index + RedIndex] = 0;

                    }
                    else if (distance > 900 && distance < 2000)
                    {
                        //we are a bit further away
                        colorFrame[index + BlueIndex] = 0;
                        colorFrame[index + GreenIndex] = 255;
                        colorFrame[index + RedIndex] = 0;
                    }
                    else if (distance > 2000)
                    {
                        //we are the farthest
                        colorFrame[index + BlueIndex] = 0;
                        colorFrame[index + GreenIndex] = 0;
                        colorFrame[index + RedIndex] = 255;
                    }


                    ////equal coloring for monochromatic histogram
                    //var intensity = CalculateIntensityFromDepth(distance);
                    //colorFrame[index + BlueIndex] = intensity;
                    //colorFrame[index + GreenIndex] = intensity;
                    //colorFrame[index + RedIndex] = intensity;

                    ////Color a player
                    if (GetPlayerIndex(depthData[depthIndex]) > 0)
                    {
                        //we are the farthest
                        colorFrame[index + BlueIndex] = 0;
                        colorFrame[index + GreenIndex] = 255;
                        colorFrame[index + RedIndex] = 255;
                    }

                    //jump two bytes at a time
                    depthIndex += 2;
                }
            }

            return colorFrame;
        }

        private byte[] GenerateMonocoloredBytes(ImageFrame imageFrame)
        {
            int height = imageFrame.Image.Height;
            int width = imageFrame.Image.Width;

            //Depth data for each pixel
            Byte[] depthData = imageFrame.Image.Bits;

            //colorFrame contains color information for all pixels in image
            //Height x Width x 4 (Red, Green, Blue, empty byte)
            Byte[] colorFrame = new byte[height * width * 4];

            //hardcoded locations to Blue, Green, Red (BGR) index positions       
            const int BlueIndex = 0;
            const int GreenIndex = 1;
            const int RedIndex = 2;

            var depthIndex = 0;
            for (var y = 0; y < height; y++)
            {
                var heightOffset = y * width;
                for (var x = 0; x < width; x++)
                {
                    var index = ((width - x - 1) + heightOffset) * 4;

                    var distance = GetDistanceWithPlayerIndex(depthData[depthIndex], depthData[depthIndex + 1]);

                    // Equal coloring for monochromatic histogram
                    var intensity = CalculateIntensityFromDepth(distance);
                    colorFrame[index + BlueIndex] = colorFrame[index + GreenIndex] = colorFrame[index + RedIndex] = intensity;

                    //jump two bytes at a time
                    depthIndex += 2;
                }
            }

            return colorFrame;
        }

        private static int GetPlayerIndex(byte firstFrame)
        {
            //returns 0 = no player, 1 = 1st player, 2 = 2nd player...
            //bitwise & on firstFrame
            return (int)firstFrame & 7;
        }


        private int GetDistanceWithPlayerIndex(byte firstFrame, byte secondFrame)
        {
            //offset by 3 in first byte to get value after player index 
            int distance = (int)(firstFrame >> 3 | secondFrame << 5);
            return distance;
        }

        const float MaxDepthDistance = 4000; // max value returned
        const float MinDepthDistance = 850; // min value returned
        const float MaxDepthDistanceOffset = MaxDepthDistance - MinDepthDistance;

        public static byte CalculateIntensityFromDepth(int distance)
        {
            //formula for calculating monochrome intensity for histogram
            return (byte)(255 - (255 * Math.Max(distance - MinDepthDistance, 0)
                / (MaxDepthDistanceOffset)));
        }


        #endregion depth



        // MICHI: It turns that I wasn't really understanding this
        private Point getDisplayPosition(Joint joint)
        {
            float depthX, depthY;

            nui.SkeletonEngine.SkeletonToDepthImage(joint.Position, out depthX, out depthY);
            // Convert to 320, 240 space
            depthX = depthX * 320;
            depthY = depthY * 240;
            int colorX, colorY;
            ImageViewArea iv = new ImageViewArea();
            // Only ImageResolution.Resolution640x480 is supported at this point
            nui.NuiCamera.GetColorPixelCoordinatesFromDepthPixel(ImageResolution.Resolution640x480, iv, (int)depthX, (int)depthY, (short)0, out colorX, out colorY);

            // Map back to skeletonCanvas.Width & skeletonCanvas.Height
            return new Point((int)(skeletonCanvas.Width * colorX / 640.0), (int)(skeletonCanvas.Height * colorY / 480));
        }
        private Point getDisplayPosition(Microsoft.Research.Kinect.Nui.Vector mainPoint)
        {
            float depthX, depthY;

            nui.SkeletonEngine.SkeletonToDepthImage(mainPoint, out depthX, out depthY);
            // Convert to 320, 240 space
            depthX = depthX * 320;
            depthY = depthY * 240;
            int colorX, colorY;
            ImageViewArea iv = new ImageViewArea();
            // Only ImageResolution.Resolution640x480 is supported at this point
            nui.NuiCamera.GetColorPixelCoordinatesFromDepthPixel(ImageResolution.Resolution640x480, iv, (int)depthX, (int)depthY, (short)0, out colorX, out colorY);

            // Map back to skeletonCanvas.Width & skeletonCanvas.Height
            return new Point((int)(skeletonCanvas.Width * colorX / 640.0), (int)(skeletonCanvas.Height * colorY / 480));
        }

        Polyline getBodySegment(Microsoft.Research.Kinect.Nui.JointsCollection joints, Brush brush, params JointID[] ids)
        {
            PointCollection points = new PointCollection(ids.Length);
            for (int i = 0; i < ids.Length; ++i)
            {
                points.Add(getDisplayPosition(joints[ids[i]]));
            }

            Polyline polyline = new Polyline();
            polyline.Points = points;
            polyline.Stroke = brush;
            polyline.StrokeThickness = 5;
            return polyline;
        }


        #region event handlers

        void nui_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Dispatcher.BeginInvoke((Action)delegate
            {
                if (colorImage.Width != 0) {
/*                    for (int i = 0; i < ColoredBytes.Length / 4; i++)
                        if (ColoredBytes[i * 4] > 100)
                            colorImage.Bits[i * 16] = colorImage.Bits[i * 16+1] = colorImage.Bits[i * 16+2] = 0;*/
                    imgCamera.Source = BitmapSource.Create(
                        colorImage.Width, colorImage.Height, 194, 194, PixelFormats.Bgr32, null, colorImage.Bits, colorImage.Width * colorImage.BytesPerPixel);
                }
                
                SkeletonFrame allSkeletons = e.SkeletonFrame;

                skeletonCanvas.Children.Clear();
                foreach (SkeletonData data in allSkeletons.Skeletons)
                {
                    if (SkeletonTrackingState.Tracked == data.TrackingState)
                    {

                        // MICHI: Draw lines to help

                        Microsoft.Research.Kinect.Nui.Vector v1 = data.Joints[JointID.Head].Position;
                        Microsoft.Research.Kinect.Nui.Vector v2 = data.Joints[JointID.Head].Position;
                        v1.Y = -1; v2.Y = 1;
                        Point p1 = getDisplayPosition(v1);
                        Point p2 = getDisplayPosition(v2);
                        DrawLimb(p1, p2);

                        #region Draw skeletonCanvas points
                        Point head = getDisplayPosition(data.Joints[JointID.Head]);
                        Point neck = getDisplayPosition(data.Joints[JointID.ShoulderCenter]);
                        Point leftShoulder = getDisplayPosition(data.Joints[JointID.ShoulderLeft]);
                        Point rightShoulder = getDisplayPosition(data.Joints[JointID.ShoulderRight]);
                        Point leftElbow = getDisplayPosition(data.Joints[JointID.ElbowLeft]);
                        Point rightElbow = getDisplayPosition(data.Joints[JointID.ElbowRight]);

                        Point leftHand = getDisplayPosition(data.Joints[JointID.HandLeft]);
                        Point rightHand = getDisplayPosition(data.Joints[JointID.HandRight]);

                        Point waist = getDisplayPosition(data.Joints[JointID.Spine]);

                        Point leftHip = getDisplayPosition(data.Joints[JointID.HipLeft]);
                        Point rightHip = getDisplayPosition(data.Joints[JointID.HipRight]);

                        Point leftKnee = getDisplayPosition(data.Joints[JointID.KneeLeft]);
                        Point rightKnee = getDisplayPosition(data.Joints[JointID.KneeRight]);


                        Point leftFoot = getDisplayPosition(data.Joints[JointID.FootLeft]);
                        Point rightFoot = getDisplayPosition(data.Joints[JointID.FootRight]);

                        Color userColor = new Color();
                        userColor = Color.FromRgb(255, 0, 0);
                        DrawCircle(head, userColor);
                        DrawCircle(neck, userColor);
                        DrawCircle(leftShoulder, userColor);
                        DrawCircle(leftElbow, userColor);
                        DrawCircle(leftHand, userColor);
                        DrawCircle(waist, userColor);
                        DrawCircle(rightShoulder, userColor);
                        DrawCircle(rightElbow, userColor);
                        DrawCircle(rightHand, userColor);
                        DrawCircle(leftHip, userColor);
                        DrawCircle(leftKnee, userColor);
                        DrawCircle(leftFoot, userColor);
                        DrawCircle(rightHip, userColor);
                        DrawCircle(rightKnee, userColor);
                        DrawCircle(rightFoot, userColor);

                        DrawLimb(head, neck);

                        DrawLimb(neck, leftShoulder);
                        DrawLimb(leftShoulder, leftElbow);
                        DrawLimb(leftElbow, leftHand);

                        DrawLimb(neck, rightShoulder);
                        DrawLimb(rightShoulder, rightElbow);
                        DrawLimb(rightElbow, rightHand);


                        DrawLimb(leftShoulder, waist);
                        DrawLimb(rightShoulder, waist);


                        DrawLimb(waist, leftHip);
                        DrawLimb(leftHip, leftKnee);
                        DrawLimb(leftKnee, leftFoot);

                        DrawLimb(waist, rightHip);
                        DrawLimb(rightHip, rightKnee);
                        DrawLimb(rightKnee, rightFoot);



                        #endregion


                        double leftAngle, rightAngle, currentHeadDistance;
                        // MICHI: New vbles
                        double bodyFloorAngle, hipsKneesHigh, headHigh;

                        string result = Recogniton.RecognizePose(data.Joints, Properties.Settings.Default.confidenceAngle,
                        Properties.Settings.Default.standPoseFactor, Properties.Settings.Default.isAutomaticChoiceAngle,
                            Properties.Settings.Default.shiftAngle, ref recognitionHeadDistance, out leftAngle, out rightAngle, out currentHeadDistance,
                            out bodyFloorAngle, out hipsKneesHigh, out headHigh);
                        if (Properties.Settings.Default.isDebug)
                        {
                            textBlock.Text = String.Format
                            ("{0}  Angle L-R: {1:F3}-{2:F3} torso:X {3:F3}, Y {4:F3}, Z {5:F3} HeadDistance I/t: {6:F3}/ {7:F3} \r\n BodyFloorAngle={8:F3} HipsKneesHigh={9:F3} HeadHigh={10:F3}",
                                result, leftAngle, rightAngle, data.Joints[JointID.HipCenter].Position.X, data.Joints[JointID.HipCenter].Position.Y,
                                data.Joints[JointID.HipCenter].Position.Z, recognitionHeadDistance, currentHeadDistance,
                                bodyFloorAngle, hipsKneesHigh, headHigh);
                        }
                        else
                            textBlock.Text = result;

                        if (logTimeChecker < System.DateTime.Now)
                        {
                            LogMessageToFile(System.String.Format(
                                    "Msg {0}.", textBlock.Text));
                            logTimeChecker = System.DateTime.Now.AddSeconds(1);
                        }

                    }
                } // for each skeleton
            });
        }

        #endregion event handlers

        #region Private state
        // MICHI: Stil not used:
//        private int minKinectCount = 1;       //0 - app is "Kinect Enabled". 1 - app "Requires Kinect".
        const int maxKinectCount = 1; //Change to 1 if you only want to view one at a time. Switching will be enabled.
        //Each Kinect needs to be in its own USB hub, otherwise it won't have enough USB bandwidth.
        //Currently only 1 Kinect per process can have SkeletalTracking working, but color and depth work for all.
        //KinectSDK TODO: enable a larger maxKinectCount (assuming your PC can dedicate a USB hub for each Kinect)
        #endregion Private state


        #region color and drawing tools
        void nui_ColorFrameReady(object sender, ImageFrameReadyEventArgs e)
        {
            Dispatcher.BeginInvoke((Action)delegate
            {

                // 32-bit per pixel, RGBA image
                colorImage = e.ImageFrame.Image;


            });
        }

        void DrawCircle(Point A, Color color)
        {
            Ellipse ellipse = new Ellipse
            {
                Fill = new SolidColorBrush(color),
                Width = 5,
                Height = 5,
                Margin = new Thickness(A.X, A.Y, 0, 0)
            };

            skeletonCanvas.Children.Add(ellipse);
        }
        void DrawLimb(Point A, Point B)
        {
            Line line = new Line();
            line.X1 = A.X;
            line.Y1 = A.Y;
            line.X2 = B.X;
            line.Y2 = B.Y;
            line.StrokeThickness = 4;
            line.Stroke = Brushes.LightGreen;



            skeletonCanvas.Children.Add(line);

        }
        #endregion color and drawing tools


        #region loging tools

        public string GetTempPath()
        {
            string path = System.Environment.GetEnvironmentVariable("TEMP");
            if (!path.EndsWith("\\")) path += "\\";
            return path;
        }

        public string GetCurrentPath()
        {
            return ".\\";
        }

        public void LogMessageToFile(string msg)
        {
            // Alternative file pre-checking
            //            if (!File.Exists("logfile.txt"))
            //                logFile = new StreamWriter("logMsgList.txt");
            //            else
            //                logFile = File.AppendText("logMsgList.txt");

            System.IO.StreamWriter sw = System.IO.File.AppendText(
                GetCurrentPath() + "MyLogFile.txt");
            try
            {
                string logLine = System.String.Format(
                        "{0:##}.{1:##}: {2}.", System.DateTime.Now.Hour, System.DateTime.Now.Minute, msg);
                sw.WriteLine(logLine);
            }
            finally
            {
                sw.Close();
            }
        }

        #endregion loging tools


        #region fall evaluation

        public double fallProbability(double headHigh, double bodyPercentage, double bodyInclination)
        {
            double probability = 0;
            double normalHigh = 0.8;
            probability = headHigh / normalHigh * 0.3; // This should be a pondered weight in the bodyPercentage
            probability += (bodyPercentage / 100) * 0.3;
            probability += (bodyInclination / 90) * 0.3;
            // Alternative: vector of probability/weight for the method to be more scalable
            return probability;
        }

        #endregion fall evaluation
    }
}
