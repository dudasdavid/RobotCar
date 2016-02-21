using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;

using Windows.Networking.Sockets;
using Windows.Networking;
using Windows.Networking.Connectivity;

using Windows.Storage.Streams;
using Windows.ApplicationModel.Core;

using System.Threading.Tasks;
using System.Threading;

using Windows.UI.Xaml.Media.Imaging;
using Windows.Graphics.Imaging;
using Windows.UI;

using System.Diagnostics;
using System.Text.RegularExpressions;
using System.Net;
using Windows.UI.Popups;

using Windows.System.Threading;


// The Blank Page item template is documented at http://go.microsoft.com/fwlink/?LinkId=234238

namespace CamViewer_1
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        StreamSocket socket = null;
        HostName hostName = null;
        DataWriter writer = null;
        DataReader reader = null;
        Stream stream = null;
        CancellationTokenSource cancellationTokenSource = null;

        string hostPort;
        string receivedData;
        string temperature;
        string battery;
        string defaultPosString;
        string faces;
        string[] facesArray = new string[15]
        {
            "0", "0", "0", "0","0", "0", "0", "0","0", "0", "0", "0","0", "0", "0"
        };
        string names;
        string[] namesArray = new string[5];

        byte[] image = new byte[76800];
        byte[] imageRGBA = new byte[76800 * 4];
        uint count = 0;

        bool connectionEnabled = true;

        uint byteCount;

        int x, y, a;
        int i = 0;

        int xPos1, xPos2, yPos1, yPos2, xDelta, yDelta;
        double joyXStartPos, joyYStartPos;
        //double posScaleFactor = 1.0;

        Color color = Color.FromArgb(255,255,0,0);


        WriteableBitmap newBmp = new WriteableBitmap(320, 240);
        WriteableBitmap newBigBmp = new WriteableBitmap(640, 480);

        RenderTargetBitmap textBmp = new RenderTargetBitmap();
        SolidColorBrush fontcolor = new SolidColorBrush(Color.FromArgb(255, 255, 255, 0));
        //IBuffer pixelBuffer = null;
        WriteableBitmap wbmp = new WriteableBitmap(1, 1);
        Point faceDestination = new Point(0,0);
        Rect nameText = new Rect(0, 0, 0, 0);

        List<TextBlock> faceCollection = new List<TextBlock>();

        Regex ValidIpAddressRegex;
        Regex ValidHostnameRegex;

        string[] regexResult;

        bool pointerIsPressed = false;

        RPhi polarCoordinates = new RPhi(0, 0);
        XY XYCoordinates = new XY(0, 0);
        DC speedsLR = new DC(0, 0);
        int scaleFactor = 50;

        int leftRefreshCtr = 0;
        int rightRefreshCtr = 0;
        int refreshInterval = 4;

        public struct RPhi
        {
            public RPhi(double r, double phi)
            {
                this.R = r;
                this.Phi = phi;
            }
            public double R;
            public double Phi;
        }

        public struct XY
        {
            public XY(double x, double y)
            {
                this.X = x;
                this.Y = y;
            }
            public double X;
            public double Y;
        }

        public struct DC
        {
            public DC(Byte l, Byte r)
            {
                this.L = l;
                this.R = r;
            }
            public Byte L;
            public Byte R;
        }

        public MainPage()
        {
            this.InitializeComponent();

            var displayRequest = new Windows.System.Display.DisplayRequest();
            displayRequest = new Windows.System.Display.DisplayRequest();
            displayRequest.RequestActive();

            int period = 200;

            ThreadPoolTimer PeriodicTimer =
                ThreadPoolTimer.CreatePeriodicTimer(TimerElapsedHandler,
                                                    TimeSpan.FromMilliseconds(period));

            PopUpCanvas.Margin = new Windows.UI.Xaml.Thickness(-85, -50, -97, -56);
            hostName = new HostName("192.168.0.100");
            hostPort = "7777";
            Windows.UI.ViewManagement.StatusBar.GetForCurrentView().HideAsync();
            SoftJoy.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
            SoftJoyBoundary.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
            HorizontalLine.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
            VerticalLine.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
            PopUpCanvas.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
            SoftJoy.Opacity = 0.2;
            faceCollection.Add(face1);
            faceCollection.Add(face2);
            faceCollection.Add(face3);
            faceCollection.Add(face4);
            faceCollection.Add(face5);

            //ValidIpAddressRegex = new Regex(@"^(([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.){3}([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])$");
            //ValidIpAddressRegex = new Regex(@"([0-9]{1,3}\.){3}[0-9]{1,3}");

            //ValidHostnameRegex = new Regex(@"^(([a-zA-Z0-9]|[a-zA-Z0-9][a-zA-Z0-9\-]*[a-zA-Z0-9])\.)*([A-Za-z0-9]|[A-Za-z0-9][A-Za-z0-9\-]*[A-Za-z0-9])$");
            ValidHostnameRegex = new Regex(@"^(([a-zA-Z]|[a-zA-Z][a-zA-Z0-9\-]*[a-zA-Z0-9])\.)*([A-Za-z]|[A-Za-z][A-Za-z0-9\-]*[A-Za-z0-9])$");

            RefreshHorizont();
            //faceCollection = new { face1, face2, face3, face4, face5 };
            this.NavigationCacheMode = NavigationCacheMode.Required;
        }

        /// <summary>
        /// Invoked when this page is about to be displayed in a Frame.
        /// </summary>
        /// <param name="e">Event data that describes how this page was reached.
        /// This parameter is typically used to configure the page.</param>
        protected override void OnNavigatedTo(NavigationEventArgs e)
        {
            // TODO: Prepare page for display here.

            // TODO: If your application contains multiple pages, ensure that you are
            // handling the hardware Back button by registering for the
            // Windows.Phone.UI.Input.HardwareButtons.BackPressed event.
            // If you are using the NavigationHelper provided by some templates,
            // this event is handled for you.
        }

        private void TimerElapsedHandler(ThreadPoolTimer timer)
        {
            
            
        }

        private RPhi XY2RPhi(double X, double Y)
        {
            RPhi tempPolarCoordinates = new RPhi(0, 0);
            double tempPhi;

            tempPolarCoordinates.R = Math.Sqrt(X * X + Y * Y);
            tempPhi = Math.Atan2(-Y, X);
            if (tempPhi >= 0) tempPolarCoordinates.Phi = tempPhi;
            else tempPolarCoordinates.Phi = tempPhi + 2 * Math.PI;
            return tempPolarCoordinates;
        }

        private XY RPhi2XY(double R, double Phi)
        {
            XY tempXY = new XY(0, 0);
            tempXY.X = R * Math.Cos(Phi);
            tempXY.Y = R * Math.Sin(Phi + Math.PI);

            return tempXY;
        }

        private DC rphi2dc(double r, double phi)
        {
            DC speedLeftRight = new DC(0, 0);

            if ((phi >= 0) && (phi < Math.PI / 2.0)) //1st quadrant
            {
                speedLeftRight.L = Convert.ToByte(r * scaleFactor + 100);
                speedLeftRight.R = Convert.ToByte(-r * scaleFactor * Math.Cos(2 * phi) + 100);
            }
            else if ((phi >= Math.PI / 2.0) && (phi < Math.PI)) //2nd quadrant
            {
                speedLeftRight.L = Convert.ToByte(-r * scaleFactor * Math.Cos(2 * phi) + 100);
                speedLeftRight.R = Convert.ToByte(r * scaleFactor + 100);
            }
            else if ((phi >= Math.PI) && (phi < 3 * Math.PI / 2.0)) //3rd quadrant
            {
                speedLeftRight.L = Convert.ToByte(-r * scaleFactor + 100);
                speedLeftRight.R = Convert.ToByte(r * scaleFactor * Math.Cos(2 * phi) + 100);
            }
            else if ((phi >= 3 * Math.PI / 2.0) && (phi <= 2 * Math.PI)) //4th quadrant
            {
                speedLeftRight.L = Convert.ToByte(r * scaleFactor * Math.Cos(2 * phi) + 100);
                speedLeftRight.R = Convert.ToByte(-r * scaleFactor + 100);
            }
            return speedLeftRight;
        }

        private async void CheckHost()
        {
            regexResult = ValidHostnameRegex.Split(IPBox.Text);
            if (regexResult[0] == "")
            {
                Debug.WriteLine("HOST");
                HostName serverHost = new HostName(IPBox.Text);
                var clientSocket = new StreamSocket();
                await clientSocket.ConnectAsync(serverHost, "http");
                var ipAddress = clientSocket.Information.RemoteAddress.DisplayName;
                hostName = new HostName(ipAddress);
            }
            else
            {
                Debug.WriteLine("IP");
                hostName = new HostName(IPBox.Text);
            }

            hostPort = PortBox.Text;
        }

        private async void ConnectButtonClick(object sender, RoutedEventArgs e)
        {
            ConnectButton.IsEnabled = false;
            if (ConnectButton.IsChecked == true)
            {
                CheckHost();

                cancellationTokenSource = new CancellationTokenSource();
                lock (this) socket = new StreamSocket();
                socket.Control.KeepAlive = true;
                socket.Control.NoDelay = true;

                try
                {
                    Debug.WriteLine("1");
                    await socket.ConnectAsync(hostName, hostPort);
                    Debug.WriteLine("2");
                    reader = new DataReader(socket.InputStream);
                    Debug.WriteLine("3");
                    writer = new DataWriter(socket.OutputStream);
                }
                catch
                {
                    MessageDialog md = new MessageDialog("Uhh oh something went wrong!", "Warning");
                    md.ShowAsync();
                    Disconnect();
                    ConnectButton.IsChecked = false;
                    ConnectButton.Content = "Connect";
                    ConnectButton.IsEnabled = true;
                    return;
                }

                connectionEnabled = true;
                ConnectButton.Content = "Disconnect";
                ConnectButton.IsEnabled = true;
                //reader.UnicodeEncoding = UnicodeEncoding.Utf8;
                //reader.InputStreamOptions = InputStreamOptions.Partial;
                PopUpCanvas.Visibility = Windows.UI.Xaml.Visibility.Collapsed;

                while (connectionEnabled)
                {
                    // wait for the available data up to 512 bytes
                    // count is the number of actually received bytes
                    try
                    {
                        count = await reader.LoadAsync(5).AsTask(cancellationTokenSource.Token);
                        byteCount = reader.UnconsumedBufferLength;
                        BufferText.Text = byteCount.ToString();
                        BufferBar.Value = byteCount;
                        receivedData = reader.ReadString(5);

                        if ((receivedData == "image") && (reader != null))
                        {
                            count = await reader.LoadAsync(76800).AsTask(cancellationTokenSource.Token);
                            reader.ReadBytes(image);

                            for (var j = 0; j < image.Length; j++)
                            {
                                imageRGBA[0 + j * 4] = image[j];
                                imageRGBA[1 + j * 4] = image[j];
                                imageRGBA[2 + j * 4] = image[j];
                                imageRGBA[3 + j * 4] = 0xff;
                            }

                            stream = newBmp.PixelBuffer.AsStream();
                            stream.Seek(0, 0);
                            stream.Write(imageRGBA, 0, imageRGBA.Length);

                            //newBigBmp = newBmp.Resize(640, 480, WriteableBitmapExtensions.Interpolation.Bilinear);
                            for (var i_faces = 0; i_faces < 5; i_faces++)
                            {
                                faceCollection[i_faces].Visibility = Windows.UI.Xaml.Visibility.Collapsed;
                            }


                            for (var i_faces = 0; i_faces < 5; i_faces++)
                            {
                                if (Convert.ToInt16(facesArray[i_faces * 3 + 2]) == 0) break;
                                x = Convert.ToInt16(facesArray[i_faces * 3 + 0]);
                                y = Convert.ToInt16(facesArray[i_faces * 3 + 1]);
                                a = Convert.ToInt16(facesArray[i_faces * 3 + 2]);
                                if (ToggleZoom.IsOn == false)
                                {
                                    newBmp.DrawRectangle(x / 2, y / 2, (x + a) / 2, (y + a) / 2, Color.FromArgb(255, 255, 255, 0));
                                    Canvas.SetLeft(faceCollection[i_faces], x / 2);
                                    Canvas.SetTop(faceCollection[i_faces], y / 2 + a / 2 + 5);
                                    faceCollection[i_faces].Text = namesArray[i_faces];
                                    //Debug.WriteLine(x / 2 + ";" + y / 2);
                                    if ((x / 2 > 0) && (x / 2 < 300) && (y / 2 > -a / 2) && (y / 2 < 230))
                                    {
                                        faceCollection[i_faces].Visibility = Windows.UI.Xaml.Visibility.Visible;
                                    }

                                }
                                else
                                {
                                    newBmp.DrawRectangle(((x - 320) * 2 + 320) / 2, ((y - 240) * 2 + 240) / 2, ((x - 320) * 2 + 320 + a * 2) / 2, ((y - 240) * 2 + 240 + a * 2) / 2, Color.FromArgb(255, 255, 255, 0));
                                    Canvas.SetLeft(faceCollection[i_faces], ((x - 320) * 2 + 320) / 2);
                                    Canvas.SetTop(faceCollection[i_faces], ((y - 240) * 2 + 240 + a * 2) / 2 + 5);
                                    faceCollection[i_faces].Text = namesArray[i_faces];
                                    //Debug.WriteLine((((x - 320) * 2 + 320) / 2) + ";" + (((y - 240) * 2 + 240 + a * 2) / 2 + 5));
                                    if ((((x - 320) * 2 + 320) / 2 > 0) && (((x - 320) * 2 + 320) / 2 < 300) && (((y - 240) * 2 + 240 + a * 2) / 2 > -a) && (((y - 240) * 2 + 240 + a * 2) / 2 < 230))
                                    {
                                        faceCollection[i_faces].Visibility = Windows.UI.Xaml.Visibility.Visible;
                                    }
                                }
                            }
                            newBmp.Invalidate();
                            CamImage.Source = newBmp;
                        }
                        else if ((receivedData == "temp_") && (reader != null))
                        {
                            count = await reader.LoadAsync(10).AsTask(cancellationTokenSource.Token);
                            temperature = reader.ReadString(10);
                            CPUText.Text = temperature.Trim('\0');
                            CPUBar.Value = Convert.ToDouble(temperature.Trim('\0'));
                        }
                        else if ((receivedData == "batt_") && (reader != null))
                        {
                            count = await reader.LoadAsync(10).AsTask(cancellationTokenSource.Token);
                            battery = reader.ReadString(10);
                            BatteryText.Text = battery.Trim('\0');
                            BatteryBar.Value = Convert.ToDouble(battery.Trim('\0'));

                        }
                        else if ((receivedData == "faces") && (reader != null))
                        {
                            count = await reader.LoadAsync(100).AsTask(cancellationTokenSource.Token);
                            faces = reader.ReadString(100);
                            facesArray = faces.Split(';');
                            //FacesCoord.Text = faces.Trim('\0');
                        }
                        else if ((receivedData == "names") && (reader != null))
                        {
                            count = await reader.LoadAsync(100).AsTask(cancellationTokenSource.Token);
                            names = reader.ReadString(100);
                            namesArray = names.Split(';');
                            aaa.Text = names.Trim('\0');
                        }
                        else if ((receivedData == "dflt_") && (reader != null))
                        {
                            count = await reader.LoadAsync(10).AsTask(cancellationTokenSource.Token);
                            defaultPosString = reader.ReadString(10);
                            //dflt.Text = dflt.Text + "\n" + defaultPosString.Trim('\0');
                            S1.Value = Convert.ToInt16(defaultPosString.Split(';')[0]);
                            S2.Value = Convert.ToInt16(defaultPosString.Split(';')[1]);
                        }
                        else if ((receivedData == "alive") && (writer != null))
                        {
                            writer.WriteString("A");
                            await writer.StoreAsync().AsTask(cancellationTokenSource.Token);
                        }
                    }
                    catch (TaskCanceledException tc)
                    {
                        //Disconnect();
                        Debug.WriteLine(tc);
                        return;
                    }
                    catch (System.NullReferenceException nr)
                    {
                        Debug.WriteLine(nr);
                        return;
                    }
                }
            }
            else
            {
                if (writer != null)
                {
                    writer.WriteString("Q");
                    await writer.StoreAsync();
                }
                Debug.WriteLine("a");
                //cancellationTokenSource.Cancel();
                connectionEnabled = false;
                Debug.WriteLine("b");
                Disconnect();
                Debug.WriteLine("c");
                ConnectButton.Content = "Connect";
                ConnectButton.IsEnabled = true;
            }
        }

        private void Disconnect()
        {
            if (cancellationTokenSource != null)
            {
                try { lock (this) cancellationTokenSource.Cancel(); }
                catch { };
                cancellationTokenSource = null;
            }
            if (reader != null)
            {
                reader.DetachStream();
                reader.Dispose();
                reader = null;
            }
            if (writer != null)
            {
                writer.DetachStream();
                writer.Dispose();
                writer = null;
            }
            if (socket != null)
            {
                lock (this) socket.Dispose();
                socket = null;
            }
        }

        private async void SendButtonClick(object sender, RoutedEventArgs e)
        {
            if (writer != null)
            {
                writer.WriteString(SendText.Text);
                await writer.StoreAsync();
            }
        }

        private async void ZoomToggled(object sender, RoutedEventArgs e)
        {
            if (ToggleZoom.IsOn == true)
            {
                if (writer != null)
                {
                    writer.WriteString("Z1");
                    await writer.StoreAsync().AsTask(cancellationTokenSource.Token);
                }
            }
            else
            {
                if (writer != null)
                {
                    writer.WriteString("Z0");
                    await writer.StoreAsync().AsTask(cancellationTokenSource.Token);
                }
            }
        }

        private async void FaceRecognitionToggled(object sender, RoutedEventArgs e)
        {
            if (ToggleFace.IsOn == true)
            {
                if (writer != null)
                {
                    writer.WriteString("F1");
                    await writer.StoreAsync().AsTask(cancellationTokenSource.Token);
                }
            }
            else
            {
                if (writer != null)
                {
                    writer.WriteString("F0");
                    await writer.StoreAsync().AsTask(cancellationTokenSource.Token);
                }
            }
        }

        private async void LEDChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            if (writer != null)
            {
                int ledval = Convert.ToInt16(LEDSlider.Value);
                writer.WriteString("L" + ledval.ToString().PadLeft(2, '0'));
                await writer.StoreAsync().AsTask(cancellationTokenSource.Token);
            }
        }

        private async void LEDChanged2(object sender, RangeBaseValueChangedEventArgs e)
        {
            if (writer != null)
            {
                int ledval = Convert.ToInt16(LEDSlider2.Value);
                writer.WriteString("R" + ledval.ToString().PadLeft(2, '0'));
                await writer.StoreAsync().AsTask(cancellationTokenSource.Token);
            }
        }

        private async void SliderChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            if (writer != null)
            {
                int S1val = Convert.ToInt16(S1.Value);
                int S2val = Convert.ToInt16(S2.Value);
                writer.WriteString("S" + S1val.ToString().PadLeft(4, '0') + S2val.ToString().PadLeft(4, '0'));
                await writer.StoreAsync().AsTask(cancellationTokenSource.Token);
            }
        }

        private async void DriveSliderChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            if (writer != null)
            {
                int LeftDriveVal = Convert.ToInt16(LeftDrive.Value);
                int RightDriveVal = Convert.ToInt16(RightDrive.Value);
                writer.WriteString("D" + LeftDriveVal.ToString().PadLeft(3, '0') + RightDriveVal.ToString().PadLeft(3, '0'));
                await writer.StoreAsync().AsTask(cancellationTokenSource.Token);

                if ((LeftDriveVal == 100) && (RightDriveVal == 100))
                {
                    for (var j = 0; j < 5; j++)
                    {
                        writer.WriteString("D" + LeftDriveVal.ToString().PadLeft(3, '0') + RightDriveVal.ToString().PadLeft(3, '0'));
                        await writer.StoreAsync().AsTask(cancellationTokenSource.Token);
                        await System.Threading.Tasks.Task.Delay(10);
                    }
                }
            }
        }

        private void ExitClick(object sender, RoutedEventArgs e)
        {
            Application.Current.Exit();
        }

        private async void StopServerClick(object sender, RoutedEventArgs e)
        {
            if (writer != null)
            {
                writer.WriteString("X");
                await writer.StoreAsync().AsTask(cancellationTokenSource.Token);
            }
            Disconnect();
        }

        private void CanvasPointerEntered(object sender, PointerRoutedEventArgs e)
        {
            //Debug.WriteLine("pointer started:" + e.GetCurrentPoint(CamImage).Position.X + e.GetCurrentPoint(CamImage).Position.Y);
            xPos1 = Convert.ToInt16(e.GetCurrentPoint(CamCanvas).Position.X);
            yPos1 = Convert.ToInt16(e.GetCurrentPoint(CamCanvas).Position.Y);
            if ((yPos1 > 200) || ((yPos1 < 50) && (xPos1 > 230)))
            {
                e.Handled = true;
                return;
            }
            //posScaleFactor = 1.0;
            if (SwipeButton.IsChecked == true)
            {
                if ((e.Pointer.PointerDeviceType == Windows.Devices.Input.PointerDeviceType.Touch) || (pointerIsPressed == true))
                {
                    joyXStartPos = e.GetCurrentPoint(CamCanvas).Position.X;
                    joyYStartPos = e.GetCurrentPoint(CamCanvas).Position.Y;
                    SoftJoy.Opacity = 0.4;
                    SoftJoy.Width = 60;
                    SoftJoy.Height = 60;
                    Canvas.SetLeft(SoftJoy, joyXStartPos - SoftJoy.Width / 2);
                    Canvas.SetTop(SoftJoy, joyYStartPos - SoftJoy.Width / 2);
                    Canvas.SetLeft(SoftJoyBoundary, joyXStartPos - SoftJoyBoundary.Width / 2);
                    Canvas.SetTop(SoftJoyBoundary, joyYStartPos - SoftJoyBoundary.Width / 2);
                    Canvas.SetLeft(HorizontalLine, joyXStartPos);
                    Canvas.SetTop(VerticalLine, joyYStartPos);

                    SoftJoy.Visibility = Windows.UI.Xaml.Visibility.Visible;
                    SoftJoyBoundary.Visibility = Windows.UI.Xaml.Visibility.Visible;
                    HorizontalLine.Visibility = Windows.UI.Xaml.Visibility.Visible;
                    VerticalLine.Visibility = Windows.UI.Xaml.Visibility.Visible;
                }
            }
            e.Handled = true;
        }

        private void CanvasPointerPressed(object sender, PointerRoutedEventArgs e)
        {
            //Debug.WriteLine("kaki");
            xPos1 = Convert.ToInt16(e.GetCurrentPoint(CamCanvas).Position.X);
            yPos1 = Convert.ToInt16(e.GetCurrentPoint(CamCanvas).Position.Y);
            if ((yPos1 > 200) || ((yPos1 < 50) && (xPos1 > 230)))
            {
                e.Handled = true;
                return;
            }
            if (SwipeButton.IsChecked == true)
            {
                joyXStartPos = e.GetCurrentPoint(CamCanvas).Position.X;
                joyYStartPos = e.GetCurrentPoint(CamCanvas).Position.Y;
                SoftJoy.Opacity = 0.4;
                SoftJoy.Width = 60;
                SoftJoy.Height = 60;
                Canvas.SetLeft(SoftJoy, joyXStartPos - SoftJoy.Width / 2);
                Canvas.SetTop(SoftJoy, joyYStartPos - SoftJoy.Width / 2);
                Canvas.SetLeft(SoftJoyBoundary, joyXStartPos - SoftJoyBoundary.Width / 2);
                Canvas.SetTop(SoftJoyBoundary, joyYStartPos - SoftJoyBoundary.Width / 2);
                Canvas.SetLeft(HorizontalLine, joyXStartPos);
                Canvas.SetTop(VerticalLine, joyYStartPos);

                SoftJoy.Visibility = Windows.UI.Xaml.Visibility.Visible;
                SoftJoyBoundary.Visibility = Windows.UI.Xaml.Visibility.Visible;
                HorizontalLine.Visibility = Windows.UI.Xaml.Visibility.Visible;
                VerticalLine.Visibility = Windows.UI.Xaml.Visibility.Visible;
            }
            pointerIsPressed = true;
            e.Handled = true;
        }

        private void CanvasPointerMoved(object sender, PointerRoutedEventArgs e)
        {
            double joyXCurrentPos, joyYCurrentPos;
            double x_temp, y_temp;
            x_temp = e.GetCurrentPoint(CamCanvas).Position.X;
            y_temp = e.GetCurrentPoint(CamCanvas).Position.Y;

            //Debug.WriteLine("{0} ; {1}", x_temp, y_temp);
            if ((y_temp > 220) || ((y_temp < 50) && (x_temp > 230) ))
            {
                e.Handled = true;
                return;
            }

            if (SwipeButton.IsChecked == false)
            {
                //Debug.WriteLine(e.Pointer.PointerDeviceType.ToString()); xPos2 = Convert.ToInt16(e.GetCurrentPoint(CamImage).Position.X);
                if ((e.Pointer.PointerDeviceType == Windows.Devices.Input.PointerDeviceType.Touch) || (pointerIsPressed == true))
                {
                    xPos2 = Convert.ToInt16(x_temp);
                    yPos2 = Convert.ToInt16(y_temp);
                    xDelta = xPos2 - xPos1;
                    yDelta = yPos2 - yPos1;
                    xPos1 = xPos2;
                    yPos1 = yPos2;


                    S1.Value = SaturateInteger(Convert.ToInt16(S1.Value - xDelta * 3), 0, 1000);
                    S2.Value = SaturateInteger(Convert.ToInt16(S2.Value + yDelta * 3), 0, 1000);
                }

            }
            else
            {
                if ((e.Pointer.PointerDeviceType == Windows.Devices.Input.PointerDeviceType.Touch) || (pointerIsPressed == true))
                {
                    joyXCurrentPos = x_temp;
                    joyYCurrentPos = y_temp;

                    //S1.Value = SaturateInteger(Convert.ToInt16(S1.Value + (joyXCurrentPos - joyXStartPos) * 0.1), 0, 1000);
                    //S2.Value = SaturateInteger(Convert.ToInt16(S2.Value + (joyYCurrentPos - joyYStartPos) * 0.1), 0, 1000);
                    XYCoordinates.X = joyXCurrentPos - joyXStartPos;
                    XYCoordinates.Y = joyYCurrentPos - joyYStartPos;
                    //Debug.WriteLine(XYCoordinates.X +";"+ XYCoordinates.Y);
                    polarCoordinates = XY2RPhi(XYCoordinates.X, XYCoordinates.Y);
                    if (polarCoordinates.R < 10)
                    {
                        polarCoordinates.R = 0;
                        XYCoordinates.X = 0;
                        XYCoordinates.Y = 0;
                    }
                    if (polarCoordinates.R > SoftJoyBoundary.Width / 2 - SoftJoy.Width / 2)
                    {
                        //e.Handled = true;
                        //return;
                        polarCoordinates.R = SoftJoyBoundary.Width / 2 - SoftJoy.Width / 2;
                        XYCoordinates = RPhi2XY(polarCoordinates.R, polarCoordinates.Phi);
                    }

                    Canvas.SetLeft(SoftJoy, joyXStartPos + XYCoordinates.X - SoftJoy.Width / 2);
                    Canvas.SetTop(SoftJoy, joyYStartPos + XYCoordinates.Y - SoftJoy.Height / 2);

                    //Canvas.SetLeft(SoftJoy, joyXCurrentPos - SoftJoy.Width / 2);
                    //Canvas.SetTop(SoftJoy, joyYCurrentPos - SoftJoy.Width / 2);


                    speedsLR = rphi2dc(polarCoordinates.R / (SoftJoyBoundary.Width / 2 - SoftJoy.Width / 2), polarCoordinates.Phi);
                    //Debug.WriteLine("{0} ; {1}", speedsLR.L, speedsLR.R);
                    LeftDrive.Value = speedsLR.L;
                    RightDrive.Value = speedsLR.R;
                }
            }

            RefreshHorizont();


            e.Handled = true;
        }

        private void CanvasPointerReleased(object sender, PointerRoutedEventArgs e)
        {
            Debug.WriteLine("kaki");
            pointerIsPressed = false;
            if (SwipeButton.IsChecked == true)
            {
                LeftDrive.Value = 100;
                RightDrive.Value = 100;
                SoftJoy.Opacity = 0.2;
                SoftJoy.Width = 40;
                SoftJoy.Height = 40;
                Canvas.SetLeft(SoftJoy, CamCanvas.Width / 2 - SoftJoy.Width / 2);
                Canvas.SetTop(SoftJoy, CamCanvas.Height / 2 - SoftJoy.Width / 2);
                Canvas.SetLeft(SoftJoyBoundary, CamCanvas.Width / 2 - SoftJoyBoundary.Width / 2);
                Canvas.SetTop(SoftJoyBoundary, CamCanvas.Height / 2 - SoftJoyBoundary.Width / 2);
                Canvas.SetLeft(HorizontalLine, CamCanvas.Width / 2);
                Canvas.SetTop(VerticalLine, CamCanvas.Height / 2);

                SoftJoy.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
                SoftJoyBoundary.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
                HorizontalLine.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
                VerticalLine.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
            }
            e.Handled = true;
        }

        private void CanvasPointerExited(object sender, PointerRoutedEventArgs e)
        {
            if (SwipeButton.IsChecked == true)
            {
                LeftDrive.Value = 100;
                RightDrive.Value = 100;
                SoftJoy.Opacity = 0.2;
                SoftJoy.Width = 40;
                SoftJoy.Height = 40;
                Canvas.SetLeft(SoftJoy, CamCanvas.Width / 2 - SoftJoy.Width / 2);
                Canvas.SetTop(SoftJoy, CamCanvas.Height / 2 - SoftJoy.Width / 2);
                Canvas.SetLeft(SoftJoyBoundary, CamCanvas.Width / 2 - SoftJoyBoundary.Width / 2);
                Canvas.SetTop(SoftJoyBoundary, CamCanvas.Height / 2 - SoftJoyBoundary.Width / 2);
                Canvas.SetLeft(HorizontalLine, CamCanvas.Width / 2);
                Canvas.SetTop(VerticalLine, CamCanvas.Height / 2);

                SoftJoy.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
                SoftJoyBoundary.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
                HorizontalLine.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
                VerticalLine.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
            }
            e.Handled = true;
        }

        private void LeftCanvasPointerStarted(object sender, PointerRoutedEventArgs e)
        {
            LeftCircle.Opacity = 0.5;
            e.Handled = true;
        }

        private void LeftCanvasPointerMoved(object sender, PointerRoutedEventArgs e)
        {
            double y = e.GetCurrentPoint(LeftPic).Position.Y;
            double tempValue = 0;
            //Debug.WriteLine("{0}", y);
            leftRefreshCtr++;

            if (y > LeftPic.Height - LeftCircle.Height / 2)
            {
                y = LeftPic.Height - LeftCircle.Height / 2;
            }
            if (y < LeftCircle.Height / 2)
            {
                y = LeftCircle.Height / 2;
            }
            if ((y > LeftPic.Height / 2 - 15) && (y < LeftPic.Height / 2 + 15))
            {
                y = LeftPic.Height / 2;
            }

            tempValue = (-1 * (y - LeftPic.Height) - LeftCircle.Height / 2) / (LeftPic.Height - LeftCircle.Height) * scaleFactor * 2 + (200 - scaleFactor * 2) / 2;
            //Debug.WriteLine("{0}", tempValue);
            LeftDrive.Value = Convert.ToInt16(tempValue);

            if (leftRefreshCtr % refreshInterval == 0)
            {
                Canvas.SetTop(LeftCircle, y - LeftCircle.Height / 2);
            }

            e.Handled = true;
        }

        private async void LeftCanvasPointerReleased(object sender, PointerRoutedEventArgs e)
        {
            LeftDrive.Value = 100;

            double yDistance = ((LeftCanvas.Height / 2 - LeftCircle.Width / 2) - Canvas.GetTop(LeftCircle)) / 20;

            for (int i = 0; i < 10; i++)
            {
                Canvas.SetTop(LeftCircle, Canvas.GetTop(LeftCircle) + yDistance);
                LeftCircle.Opacity -= 0.01;
                await System.Threading.Tasks.Task.Delay(10);
            }

            Canvas.SetTop(LeftCircle, LeftCanvas.Height / 2 - LeftCircle.Width / 2);
            LeftCircle.Opacity = 0.3;

            e.Handled = true;
        }

        private void RightCanvasPointerStarted(object sender, PointerRoutedEventArgs e)
        {
            RightCircle.Opacity = 0.5;
            e.Handled = true;
        }

        private void RightCanvasPointerMoved(object sender, PointerRoutedEventArgs e)
        {
            double y = e.GetCurrentPoint(RightPic).Position.Y;
            double tempValue = 0;
            //Debug.WriteLine("{0}", y);
            rightRefreshCtr++;

            if (y > RightPic.Height - RightCircle.Height / 2)
            {
                y = RightPic.Height - RightCircle.Height / 2;
            }
            if (y < RightCircle.Height / 2)
            {
                y = RightCircle.Height / 2;
            }
            if ((y > RightPic.Height / 2 - 15) && (y < RightPic.Height / 2 + 15))
            {
                y = RightPic.Height / 2;
            }

            tempValue = (-1 * (y - RightPic.Height) - RightCircle.Height / 2) / (RightPic.Height - RightCircle.Height) * scaleFactor * 2 + (200 - scaleFactor * 2) / 2;
            //Debug.WriteLine("{0}", tempValue);
            RightDrive.Value = Convert.ToInt16(tempValue);

            if (rightRefreshCtr % refreshInterval == 0)
            {
                Canvas.SetTop(RightCircle, y - RightCircle.Height / 2);
            }

            e.Handled = true;
        }

        private async void RightCanvasPointerReleased(object sender, PointerRoutedEventArgs e)
        {

            RightDrive.Value = 100;

            double yDistance = ((RightCanvas.Height / 2 - RightCircle.Width / 2) - Canvas.GetTop(RightCircle)) / 20;

            for (int i = 0; i < 10; i++)
            {
                Canvas.SetTop(RightCircle, Canvas.GetTop(RightCircle) + yDistance);
                RightCircle.Opacity -= 0.01;
                await System.Threading.Tasks.Task.Delay(10);
            }

            Canvas.SetTop(RightCircle, RightCanvas.Height / 2 - RightCircle.Width / 2);
            RightCircle.Opacity = 0.3;


            e.Handled = true;
        }

        private void CenterCameraClick(object sender, RoutedEventArgs e)
        {
            S1.Value = 501;
            S1.Value = 500;
            S2.Value = 800;

            RefreshHorizont();
        }

        private void RefreshHorizont()
        {
            Canvas.SetLeft(CenterHorizontalLine1, 160 - (500 - S1.Value) * 0.25);
            Canvas.SetLeft(CenterVerticalLine1, 180 - (500 - S1.Value) * 0.25);
            //Canvas.SetLeft(CenterVerticalLine2, -180 - (500 - S1.Value) * 0.3);
            Canvas.SetLeft(CenterCircle, 157 - (500 - S1.Value) * 0.25);
            CenterVerticalLine1.Width = 140 + (500 - S1.Value) * 0.25;
            CenterVerticalLine2.Width = 140 - (500 - S1.Value) * 0.25;

            Canvas.SetTop(CenterHorizontalLine1, 180 + (500 - S2.Value) * 0.1);
            Canvas.SetTop(CenterVerticalLine1, 160 + (500 - S2.Value) * 0.1);
            Canvas.SetTop(CenterVerticalLine2, 160 + (500 - S2.Value) * 0.1);
            Canvas.SetTop(CenterCircle, 157 + (500 - S2.Value) * 0.1);
            CenterHorizontalLine1.Height = 60 - (500 - S2.Value) * 0.1;

            Canvas.SetLeft(CenterArc, 140 - (500 - S1.Value) * 0.25);
            Canvas.SetTop(CenterArc, 160 + (500 - S2.Value) * 0.1);
        }


        private void PowerSwitch(object sender, RoutedEventArgs e)
        {
            if (HighSpeedButton.IsChecked == true)
            {
                HighSpeedButton.Content = "HiPower ON";
                scaleFactor = 100;
            }
            else
            {
                HighSpeedButton.Content = "HiPower OFF";
                scaleFactor = 50;
            }
        }

        public int SaturateInteger(int val, int min, int max) {
            if (val > max) return max;
            else if (val < min) return min;
            else return val;
        }

        private void DoubleTapOnImage(object sender, DoubleTappedRoutedEventArgs e)
        {
            //Debug.WriteLine("kaka");
            ToggleZoom.IsOn ^= true;
            e.Handled = true;
        }

        private async void LeftDriveLost(object sender, PointerRoutedEventArgs e)
        {
            LeftDrive.Value = 100;
            if (writer != null)
            {
                int LeftDriveVal = Convert.ToInt16(LeftDrive.Value);
                int RightDriveVal = Convert.ToInt16(RightDrive.Value);
                writer.WriteString("D" + LeftDriveVal.ToString().PadLeft(3, '0') + RightDriveVal.ToString().PadLeft(3, '0'));
                await writer.StoreAsync().AsTask(cancellationTokenSource.Token);
            }
            e.Handled = true;
        }

        private async void RightDriveLost(object sender, PointerRoutedEventArgs e)
        {
            RightDrive.Value = 100;
            if (writer != null)
            {
                int LeftDriveVal = Convert.ToInt16(LeftDrive.Value);
                int RightDriveVal = Convert.ToInt16(RightDrive.Value);
                writer.WriteString("D" + LeftDriveVal.ToString().PadLeft(3, '0') + RightDriveVal.ToString().PadLeft(3, '0'));
                await writer.StoreAsync().AsTask(cancellationTokenSource.Token);
            }
            e.Handled = true;
        }

        private void ShowMenuClick(object sender, RoutedEventArgs e)
        {
            if (PopUpCanvas.Visibility == Windows.UI.Xaml.Visibility.Visible)
            {
                PopUpCanvas.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
            }
            else
            {
                PopUpCanvas.Visibility = Windows.UI.Xaml.Visibility.Visible;
            }
        }

        private void CheckKey(object sender, KeyRoutedEventArgs e)
        {
            if (e.Key == Windows.System.VirtualKey.Enter)
            {
                this.IsEnabled = false;
                this.IsEnabled = true;
                hostName = new HostName(IPBox.Text);
                hostPort = PortBox.Text;
                //PopUpCanvas.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
            }
        }

        private void SwipeClick(object sender, RoutedEventArgs e)
        {
            if (SwipeButton.IsChecked == true)
            {
                //SoftJoy.Visibility = Windows.UI.Xaml.Visibility.Visible;
                //SoftJoyBoundary.Visibility = Windows.UI.Xaml.Visibility.Visible;
                //HorizontalLine.Visibility = Windows.UI.Xaml.Visibility.Visible;
                //VerticalLine.Visibility = Windows.UI.Xaml.Visibility.Visible;
                SwipeButton.Content = "CAM control";
            }
            else
            {
                SoftJoy.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
                SoftJoyBoundary.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
                HorizontalLine.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
                VerticalLine.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
                SwipeButton.Content = "CAR control";
            }
            PopUpCanvas.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
        }
    }
}
