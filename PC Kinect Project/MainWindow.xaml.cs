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

        // Parameters
        const float MaxDepthDistance = 4000; // max value returned
        const float MinDepthDistance = 850; // min value returned
        const float MaxDepthDistanceOffset = MaxDepthDistance - MinDepthDistance;

        // External (reality) parameters
        int kinectAngle = 0;
        double kinectBaseInclination = 0;

        // Public vble initializations for MainWindow
        BackgroundWorker _worker = new BackgroundWorker();
        Runtime nui; //Kinect Runtime

        // Image bitmaps and frames
        PlanarImage colorImage = new PlanarImage();
        ImageFrame currentDepthImageFrame = new ImageFrame();
        byte[] initialMaxDepthMatrix, initialMinDepthMatrix, currentDepthMatrix;

        // Log and process variables
        double recognitionHeadDistance = 0;
        DateTime lastTime = DateTime.MaxValue;
        DateTime logTimeChecker = DateTime.MaxValue;
        int depthTrainingImages = 0;
        int depthMode012 = 0;

        // MICHI: Temp test
        int minX = 1000;
        int minY = 1000;
        int maxX = 0;
        int maxY = 0;
        Point cloudCentroid = new Point(0, 0);

        // Children windows
        DebugWindow dW = new DebugWindow();

        // Every joint is assigned a color forming a "joint-colour" dictionary
        Dictionary<JointID, Brush> jointColors = new Dictionary<JointID, Brush>() { 
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
            dW.Show();
            this.Focus();
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            // Cleanup
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
                Smoothing = 0.5f, //0.75f
                Correction = 0.25f, //0.5f, //0.0f
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
            { // If there are Kinects, the setup begins
                // Always use only first Kinect
                nui = Runtime.Kinects[0];
                CompositionTarget.Rendering += new EventHandler(CompositionTarget_Rendering);

                // Initialize skeletal tracking if possible
                try
                {
                    nui.Initialize(RuntimeOptions.UseDepthAndPlayerIndex | RuntimeOptions.UseSkeletalTracking | RuntimeOptions.UseColor);
                    //nui.Initialize(RuntimeOptions.UseSkeletalTracking);
                    //                    enableSmoothness(); // MICHI: smoothness deactivated
                }
                catch (InvalidOperationException)
                {
                    System.Windows.MessageBox.Show("Runtime initialization failed. Please make sure Kinect device is plugged in.");
                    return;
                }

                // Initialize color and depth video streams if possible
                try
                {
                    nui.VideoStream.Open(ImageStreamType.Video, 2, ImageResolution.Resolution640x480, ImageType.Color);
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
                nui.DepthFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_FirstDepthFrame);

                logTimeChecker = lastTime = DateTime.Now;
                LogMessageToFile(System.String.Format(
                        "\r\n\t{0:G}: PC Kinect Project application has been launched.", System.DateTime.Now));


            }
        }


        void CompositionTarget_Rendering(object sender, EventArgs e)
        {
            // This had always been commented... *-)
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

        #region depth events
        // "nui_FirstDepthFrame" reads the first image to first prepare the background segmentation
        void nui_FirstDepthFrame(object sender, ImageFrameReadyEventArgs e)
        {
            nui.DepthFrameReady -= new EventHandler<ImageFrameReadyEventArgs>(nui_FirstDepthFrame);

            // Disable the calibration button while calibrating and reset counter
            button1.IsEnabled = false;
            depthTrainingImages = 0;

            currentDepthImageFrame = e.ImageFrame;
            initialMaxDepthMatrix = GenerateDepthBytes(e.ImageFrame);
            initialMinDepthMatrix = GenerateDepthBytes(e.ImageFrame);
            currentDepthMatrix = GenerateDepthBytes(e.ImageFrame);


            nui.DepthFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_DepthConfigStage);
        }

        // "nui_DepthConfigStage" reads the first 100 images and keeps the greater and the lower depth values for the background segmentation
        void nui_DepthConfigStage(object sender, ImageFrameReadyEventArgs e)
        {
            currentDepthMatrix = GenerateDepthBytes(e.ImageFrame);
            currentDepthImageFrame = e.ImageFrame;

            #region updateMiniMaxValues
            int matrixLength = currentDepthMatrix.Length;

            for (var i = 0; i < matrixLength; i++)
            {
                if (currentDepthMatrix[i] > initialMaxDepthMatrix[i])
                    initialMaxDepthMatrix[i] = currentDepthMatrix[i];
                else if (currentDepthMatrix[i] < initialMinDepthMatrix[i])
                    initialMinDepthMatrix[i] = currentDepthMatrix[i];
            }

            #endregion updateMiniMaxValues


            if (++depthTrainingImages > 99)
            {
                nui.DepthFrameReady -= new EventHandler<ImageFrameReadyEventArgs>(nui_DepthConfigStage);
                // Enable the calibration button after calibrating
                button1.IsEnabled = true;
                nui.DepthFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_DepthFrameReady);
            }

        }

        void nui_DepthFrameReady(object sender, ImageFrameReadyEventArgs e)
        {
            currentDepthImageFrame = e.ImageFrame;
            currentDepthMatrix = GenerateDepthBytes(e.ImageFrame);
        }
        private byte[] GenerateMonocoloredBytes(ImageFrame imageFrame)
        {
            int height = imageFrame.Image.Height;
            int width = imageFrame.Image.Width;

            //Depth data for each pixel
            Byte[] depthRawData = imageFrame.Image.Bits;

            //depthFrame contains color information for all pixels in image
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

                    var distance = GetDistanceWithPlayerIndex(depthRawData[depthIndex], depthRawData[depthIndex + 1]);

                    // Equal coloring for monochromatic histogram
                    var intensity = CalculateIntensityFromDepth(distance);
                    colorFrame[index + BlueIndex] = colorFrame[index + GreenIndex] = colorFrame[index + RedIndex] = intensity;

                    //jump two bytes at a time
                    depthIndex += 2;
                }
            }

            return colorFrame;
        }

        #endregion depth events

        private byte[] GenerateDepthBytes(ImageFrame imageFrame)
        {
            int height = imageFrame.Image.Height;
            int width = imageFrame.Image.Width;

            //Depth data for each pixel
            Byte[] depthRawData = imageFrame.Image.Bits;

            //depthFrame contains color information for all pixels in image
            //Height x Width x 4 (Red, Green, Blue, empty byte)
            Byte[] depthFrame = new byte[height * width];

            var depthIndex = 0;
            for (var y = 0; y < height; y++)
            {
                var heightOffset = y * width;
                for (var x = 0; x < width; x++)
                {
                    var index = (x + heightOffset);
                    var distance = GetDistanceWithPlayerIndex(depthRawData[depthIndex], depthRawData[depthIndex + 1]);

                    // Equal coloring for monochromatic histogram
                    depthFrame[index] = CalculateIntensityFromDepth(distance);

                    //jump two bytes at a time
                    depthIndex += 2;
                }
            }

            return depthFrame;
        }

        private int GetDistanceWithPlayerIndex(byte firstFrame, byte secondFrame)
        {
            //offset by 3 in first byte to get value after player index 
            int distance = (int)(firstFrame >> 3 | secondFrame << 5);
            return distance;
        }

        private byte CalculateIntensityFromDepth(int distance)
        {
            //formula for calculating monochrome intensity for histogram
            return (byte)(255 - (255 * Math.Max(distance - MinDepthDistance, 0)
                / (MaxDepthDistanceOffset)));
        }

        private static int GetPlayerIndex(byte firstFrame)
        {
            //returns 0 = no player, 1 = 1st player, 2 = 2nd player...
            //bitwise & on firstFrame
            return (int)firstFrame & 7;
        }

        private void processBackground(int bpp, int colorImageLength, PlanarImage depthImage)
        {
            // Old escalable and cool but yet to fix way
            /*
                        for (int i = 0; i < colorImageLength; i++)
                        {

                            int x = i % colorImage.Width;
                            int y = i / colorImage.Width;
                            // Resize :
                            x = (int)(x * ((double)depthImage.Width / colorImage.Width));
                            y = (int)(y * ((double)depthImage.Height / colorImage.Height));
                            int index = y * depthImage.Width + x;
                            if ((currentDepthMatrix[index] > 250))
                                currentDepthMatrix[index] = 0;
                            if ((currentDepthMatrix[index] > 250) || ((currentDepthMatrix[index] < initialMaxDepthMatrix[index] + 10) && (currentDepthMatrix[index] > initialMinDepthMatrix[index] - 10)))
                            {
                                colorImage.Bits[i * bpp] = colorImage.Bits[i * bpp + 1] = colorImage.Bits[i * bpp + 2] = 0;
                            }
                        }
             */
            Point tempCentroid = new Point(0, 0);
            int numberOfPointsInCloud = 0;
            // New (yet-more) cool and robust seem
            int depthMaxIndex = (currentDepthImageFrame.Image.Width * currentDepthImageFrame.Image.Height);
            for (int i = 0; i < depthMaxIndex; i++)
            {
                if (GetPlayerIndex(currentDepthImageFrame.Image.Bits[i * 2]) > 0)
                {
                    // Adding the centroid calculation
                    numberOfPointsInCloud++;
                    tempCentroid.X += i % currentDepthImageFrame.Image.Width;
                    tempCentroid.Y += i / currentDepthImageFrame.Image.Width;
                    currentDepthMatrix[i] = 255;
                }
                else
                {
                    currentDepthMatrix[i] = 0;
                    Point colorPoint = getColorMatrixPosition(i);
                    for (int j = -1; j < 2; j++)
                    {
                        for (int k = 0; k < 3; k++)
                        {
                            colorImage.Bits[(int)((colorPoint.X + j + colorPoint.Y * colorImage.Width) * bpp) + k] = 0;
                        }
                    }
                }
                /*
                // MICHI: REALLY VERY INTERESTING: with this it is possible to see the matrix which corresponds to the color image!!!
                    Point colorPoint = getColorMatrixPosition(i);
                colorImage.Bits[(int)((colorPoint.X + colorPoint.Y * colorImage.Width) * bpp)] = 0;
                colorImage.Bits[(int)((colorPoint.X + colorPoint.Y * colorImage.Width) * bpp)+1] = 0;
                colorImage.Bits[(int)((colorPoint.X + colorPoint.Y * colorImage.Width) * bpp)+2] = 0;
                 */
            }
            /*
                        depthMode012 = 0;
                        switch (depthMode012)
                        {
                            case 0:
                                        dW.debugImage.Source = BitmapSource.Create(320, 240, 96, 96, PixelFormats.Gray8, null,
                                            currentDepthMatrix, 320 * PixelFormats.Gray8.BitsPerPixel / 8);
                                        break;
                            case 1:
                                        dW.debugImage.Source = BitmapSource.Create(320, 240, 96, 96, PixelFormats.Gray8, null,
                                            initialMinDepthMatrix, 320 * PixelFormats.Gray8.BitsPerPixel / 8);
                                        break;
                            case 2:
                                        dW.debugImage.Source = BitmapSource.Create(320, 240, 96, 96, PixelFormats.Gray8, null,
                                            initialMaxDepthMatrix, 320 * PixelFormats.Gray8.BitsPerPixel / 8);
                                        break;
                            default:
                                break;
                        }
             */

            // If the centroid have been calculated:
            if (numberOfPointsInCloud > 0)
            {
                tempCentroid.X = tempCentroid.X / numberOfPointsInCloud;
                tempCentroid.Y = tempCentroid.Y / numberOfPointsInCloud;
                cloudCentroid = getDisplayPosition((float)tempCentroid.X / 320, (float)tempCentroid.Y / 240);
            }
        }


        #endregion depth


        #region skeleton management

        // Receives the just the depth pixel index
        private Point getColorMatrixPosition(int depthIndex)
        {
            // Convert to 320, 240 space
            float depthX = depthIndex % 320;
            float depthY = depthIndex / 320;
            int colorX, colorY;
            ImageViewArea iv = new ImageViewArea();

            // Only ImageResolution.Resolution640x480 is supported at this point
            nui.NuiCamera.GetColorPixelCoordinatesFromDepthPixel(ImageResolution.Resolution640x480, iv, (int)depthX, (int)depthY, (short)0, out colorX, out colorY);

            // Map back to skeletonCanvas.Width & skeletonCanvas.Height
            return new Point((int)(colorX), (int)(colorY));
        }

        // Receives the RAW depthX and Y (no the 320x240 one)
        private Point getDisplayPosition(float depthX, float depthY)
        {
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


        private Point getDisplayPosition(Joint joint)
        {
            float depthX, depthY;
            nui.SkeletonEngine.SkeletonToDepthImage(joint.Position, out depthX, out depthY);

            return getDisplayPosition(depthX, depthY);
        }

        private Point getDisplayPosition(Microsoft.Research.Kinect.Nui.Vector mainPoint)
        {
            float depthX, depthY;

            nui.SkeletonEngine.SkeletonToDepthImage(mainPoint, out depthX, out depthY);
            // Convert to 320, 240 space
            depthX = depthX * 320; // MICHI: WATCH OUT HERE!!!! this are the depth X/Y coordinates first in the 0-1 interval
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

        void nui_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Dispatcher.BeginInvoke((Action)delegate
            {
                if (currentDepthMatrix != null && colorImage.Width != 0)
                {
                    processBackground(colorImage.BytesPerPixel, colorImage.Height * colorImage.Width, currentDepthImageFrame.Image);
                    imgCamera.Source = BitmapSource.Create(
                      colorImage.Width, colorImage.Height, 194, 194, PixelFormats.Bgr32, null, colorImage.Bits, colorImage.Width * colorImage.BytesPerPixel);
                    dW.debugImage.Source = BitmapSource.Create(320, 240, 96, 96, PixelFormats.Gray8, null,
                        currentDepthMatrix, 320 * PixelFormats.Gray8.BitsPerPixel / 8);
                }

                SkeletonFrame allSkeletons = e.SkeletonFrame;

                skeletonCanvas.Children.Clear();
                foreach (SkeletonData data in allSkeletons.Skeletons)
                {
                    if (SkeletonTrackingState.Tracked == data.TrackingState)
                    {

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
                        Color fallenColor = new Color();
                        userColor = Color.FromRgb(0, 0, 255);
                        fallenColor = Color.FromRgb(255, 0, 0);
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

                        // MICHI: temp test
                        /*                        if (minX > rightHand.X)
                                                    minX = (int)rightHand.X;
                                                else if (maxX < rightHand.X)
                                                    maxX = (int)rightHand.X;
                                                if (minY > rightHand.Y)
                                                    minY = (int)rightHand.Y;
                                                else if (maxY < rightHand.Y)
                                                    maxY = (int)rightHand.Y;
                                                Point leftUpperPoint = new Point(minX, maxY);
                                                Point rightUpperPoint = new Point(maxX, maxY);
                                                Point leftLowerPoint = new Point(minX, minY);
                                                Point rightLowerPoint = new Point(maxX, minY);
                                                DrawLimb(leftUpperPoint, rightUpperPoint);
                                                DrawLimb(leftLowerPoint, rightLowerPoint);
                                                */
                        Point lfUpPt = getDisplayPosition(0, 0);
                        Point rgDownPt = getDisplayPosition(1, 1);
                        DrawLimb(lfUpPt, rgDownPt);
                        DrawCentroid(cloudCentroid, Color.FromRgb(255, 0, 0));


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

        #endregion skeleton management

        #region Private state
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

        void DrawCentroid(Point A, Color color)
        {
            Ellipse ellipse = new Ellipse
            {
                Fill = new SolidColorBrush(color),
                Width = 15,
                Height = 15,
                Margin = new Thickness(A.X, A.Y, 0, 0)
            };
            skeletonCanvas.Children.Add(ellipse);
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
            double limit = 350; // MICHI: testing; Between 250 and 500??
            Line line = new Line();
            line.X1 = A.X;
            line.Y1 = A.Y;
            line.X2 = B.X;
            line.Y2 = B.Y;
            line.StrokeThickness = 4;
            if (A.Y > limit && B.Y > limit)
                line.Stroke = Brushes.Red;
            else
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

        #region communication

        // Function for inquiring the state of the shimmer.
        // Returns: 0 if everything is ok (that is correctly read and no fall)
        //          1 if there was a fall in the las 'T' time (T is an external parameter of the shimmer)
        //          -1 if there was an error while trying to find out the state
        public int readShimmerState()
        {
            try
            {
                return shimmerStateFromFile();
            }
            catch (Exception exception)
            {
                Console.WriteLine("Exception in readShimmerState: {0}", exception.ToString());
                return -1;
            }
            return -1; // Unreachable line; it is there for safety after maintenance
        }

        // Function for reading the shimmer state file.
        // Returns: 0 if everything is ok (that is correctly read and no fall)
        //          1 if there was a fall in the las 'T' time (T is an external parameter of the shimmer)
        //          -1 if there was an error while trying to find out the state
        public int shimmerStateFromFile()
        {
            byte[] stateFileData;
            try
            {
                stateFileData = ReadFile(".\\shimmerStateExample.txt");
            }
            catch (Exception exception)
            {
                Console.WriteLine("Exception in shimmerStateFromFile: {0}", exception.ToString());
                return -1;
            }
            if (stateFileData[0] == 0 || stateFileData[0] == '0')
                return 0;
            else if (stateFileData[0] == 1 || stateFileData[0] == '1')
                return 1;
            return -1;
        }

        // Generic file reader method
        // I'll try to use this for better modularity even though it would be easier to implement the shimmer state from here
        // Returns: a buffer with (hopefully) all the data in the file
        public byte[] ReadFile(string filePath)
        {
            byte[] buffer;
            FileStream fileStream = new FileStream(filePath, FileMode.Open, FileAccess.Read);
            try
            {
                int length = (int)fileStream.Length;  // get file length
                buffer = new byte[length];            // create buffer
                int count;                            // actual number of bytes read
                int sum = 0;                          // total number of bytes read

                // read until Read method returns 0 (end of the stream has been reached)
                while ((count = fileStream.Read(buffer, sum, length - sum)) > 0)
                    sum += count;  // sum is a buffer offset for next reading
            }
            finally
            {
                fileStream.Close();
            }
            return buffer;
        }

        // Function for calling the caregiver throw Skype
        // In a final version this should be done with a video call but that is out of the current scope
        // Returns: true if the application was correctly executed (no guarantee that the call was a success)
        //          false if the execution failed.
        public bool callCaregiver()
        {
            System.Diagnostics.Process proc = new System.Diagnostics.Process();
            proc.EnableRaisingEvents = false;
            proc.StartInfo.FileName = "C:\\Program Files (x86)\\Skype\\Phone\\Skype.exe";
            proc.StartInfo.Arguments = "/callto:echo123";
            try
            {
                proc.Start();
            }
            catch (Exception exception)
            {
                Console.WriteLine("Exception in callCaregiver: {0}", exception.ToString());
                return false;
            }
            return true;
        }

        #endregion communication

        private void button2_Click(object sender, RoutedEventArgs e)
        {
            angleSliderPanel asp = new angleSliderPanel(nui, kinectBaseInclination);
            if (asp.ShowDialog().HasValue)
            {
                kinectAngle = (int)asp.selectedAngle;
                kinectBaseInclination = asp.selectedBaseInclination;
            }
        }

        private void button3_Click(object sender, RoutedEventArgs e)
        {
            FallRecognition.App.Current.Shutdown(0);
        }

        private void button1_Click(object sender, RoutedEventArgs e)
        {
            nui.DepthFrameReady -= new EventHandler<ImageFrameReadyEventArgs>(nui_DepthFrameReady);
            nui.DepthFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_FirstDepthFrame);
        }

        private void button4_Click(object sender, RoutedEventArgs e)
        {
            MessageBox.Show("In the file, it was read the number: " + readShimmerState().ToString());
            /*            System.Diagnostics.Process proc = new System.Diagnostics.Process();
                        proc.EnableRaisingEvents = false;
                        proc.StartInfo.FileName = "C:\\Program Files (x86)\\Skype\\Phone\\Skype.exe";
                        proc.StartInfo.Arguments = "/callto:echo123";
                        try
                        {
                            proc.Start();
                        }
                        catch (Exception exception)
                        {
                            Console.WriteLine("Exception: {0}", exception.ToString());
                        }
                        */
        }
    }
}
