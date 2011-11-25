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
        public angleSliderPanel(Runtime newNui, int iniAngle)
        {
            nui = newNui;
            InitializeComponent();
            label1.Content = iniAngle.ToString();
            selectedAngle = iniAngle;
            slider1.Value = selectedAngle;
        }

        private void button1_Click(object sender, RoutedEventArgs e)
        {
            if (nui != null && Math.Abs(slider1.Value) <  28) // The .Value is double checked but is very important.
            {
                //Set angle to slider1 value
                nui.NuiCamera.ElevationAngle = (int)slider1.Value;
                selectedAngle = (int)slider1.Value;
                this.Close();
            }
            else
                MessageBox.Show("Null reference to Kinect runtime");
        }

        private void slider1_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            label1.Content = ((int)(slider1.Value)).ToString();
        }
    }
}
