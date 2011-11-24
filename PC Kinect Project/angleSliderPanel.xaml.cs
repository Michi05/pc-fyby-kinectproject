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
        public angleSliderPanel(Runtime newNui)
        {
            nui = newNui;
            InitializeComponent();
        }

        private void button1_Click(object sender, RoutedEventArgs e)
        {
            if (nui != null)
            {
                //Set angle to slider1 value
                nui.NuiCamera.ElevationAngle = (int)slider1.Value;
            }
            else
                MessageBox.Show("Null reference to Kinect runtime");
        }
    }
}
