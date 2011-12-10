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
using System.Windows.Shapes;
using Microsoft.Research.Kinect.Nui;

namespace FallRecognition
{
    /// <summary>
    /// Interaction logic for angleSliderPanel.xaml
    /// </summary>
    public partial class angleSliderPanel : Window
    {
        Runtime nui;
        public double selectedAngle;
        public double selectedBaseInclination;
        public angleSliderPanel(Runtime newNui, double kinectBaseInclination)
        {
            nui = newNui;
            InitializeComponent();
            // Update internal values
            selectedAngle = nui.NuiCamera.ElevationAngle;
            selectedBaseInclination = kinectBaseInclination;
            // Update interface
            slider1.Value = selectedAngle;
            label1.Content = selectedAngle.ToString();
            textBox1.Text = kinectBaseInclination.ToString();
        }




        private void button1_Click(object sender, RoutedEventArgs e)
        {
            if (nui == null)
                MessageBox.Show("Null reference to Kinect runtime");
            else if (Math.Abs(slider1.Value) < 28) // The .Value is double checked but is very important.
            {
                //Set angle to slider1 value
                nui.NuiCamera.ElevationAngle = (int)slider1.Value;
                selectedAngle = (int)slider1.Value;
                // Parse possible new base inclination
                int parseResult = 0;
                if (int.TryParse(textBox1.Text, out parseResult))
                    selectedBaseInclination = parseResult;

                this.Close();
            }
            else
                MessageBox.Show("The kinect angle must between -27 and 27");
        }

        private void slider1_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            label1.Content = ((int)(slider1.Value)).ToString();
        }
    }
}
