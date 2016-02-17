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

using System.Diagnostics;
using System.Threading.Tasks;

using Windows.UI.Xaml.Media.Imaging;
using Windows.Graphics.Imaging;
using System.Threading;

using Windows.UI;
using System.Text.RegularExpressions;
using System.Net;
using Windows.UI.Popups;

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
        CancellationTokenSource cancellationTokenSource = new CancellationTokenSource();
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

        Color color = Color.FromArgb(255, 255, 0, 0);


        WriteableBitmap newBmp = new WriteableBitmap(320, 240);
        WriteableBitmap newBigBmp = new WriteableBitmap(640, 480);

        RenderTargetBitmap textBmp = new RenderTargetBitmap();
        SolidColorBrush fontcolor = new SolidColorBrush(Color.FromArgb(255, 255, 255, 0));
        //IBuffer pixelBuffer = null;
        WriteableBitmap wbmp = new WriteableBitmap(1, 1);
        Point faceDestination = new Point(0, 0);
        Rect nameText = new Rect(0, 0, 0, 0);

        List<TextBlock> faceCollection = new List<TextBlock>();

        Regex ValidIpAddressRegex;
        Regex ValidHostnameRegex;

        string[] regexResult;

        bool pointerIsPressed = false;

        public  MainPage()
        {
            this.InitializeComponent();
            hostName = new HostName("192.168.0.100");
            hostPort = "7777";
            //Windows.UI.ViewManagement.StatusBar.GetForCurrentView().HideAsync();
            SoftJoy.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
            HorizontalLine.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
            VerticalLine.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
            PopUpCanvas.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
            faceCollection.Add(face1);
            faceCollection.Add(face2);
            faceCollection.Add(face3);
            faceCollection.Add(face4);
            faceCollection.Add(face5);

            //ValidIpAddressRegex = new Regex(@"^(([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.){3}([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])$");
            //ValidIpAddressRegex = new Regex(@"([0-9]{1,3}\.){3}[0-9]{1,3}");

            //ValidHostnameRegex = new Regex(@"^(([a-zA-Z0-9]|[a-zA-Z0-9][a-zA-Z0-9\-]*[a-zA-Z0-9])\.)*([A-Za-z0-9]|[A-Za-z0-9][A-Za-z0-9\-]*[A-Za-z0-9])$");
            ValidHostnameRegex = new Regex(@"^(([a-zA-Z]|[a-zA-Z][a-zA-Z0-9\-]*[a-zA-Z0-9])\.)*([A-Za-z]|[A-Za-z][A-Za-z0-9\-]*[A-Za-z0-9])$");


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
                                    Canvas.SetLeft(faceCollection[i_faces], x / 2 * CamCanvas.Width/320);
                                    Canvas.SetTop(faceCollection[i_faces], (y / 2 + a / 2) * CamCanvas.Height / 240 + 5);
                                    faceCollection[i_faces].Text = namesArray[i_faces];
                                    faceCollection[i_faces].Visibility = Windows.UI.Xaml.Visibility.Visible;
                                    /*
                                    if ((x / 2 > 0) && (x / 2 < 300) && (y / 2 > -a / 2) && (y / 2 < 230))
                                    {
                                        faceCollection[i_faces].Visibility = Windows.UI.Xaml.Visibility.Visible;
                                    }
                                    */

                                }
                                else
                                {
                                    newBmp.DrawRectangle(((x - 320) * 2 + 320) / 2, ((y - 240) * 2 + 240) / 2, ((x - 320) * 2 + 320 + a * 2) / 2, ((y - 240) * 2 + 240 + a * 2) / 2, Color.FromArgb(255, 255, 255, 0));
                                    Canvas.SetLeft(faceCollection[i_faces], ((x - 320) * 2 + 320) / 2 * CamCanvas.Width / 320);
                                    Canvas.SetTop(faceCollection[i_faces], ((y - 240) * 2 + 240 + a * 2) / 2 * CamCanvas.Height / 240 + 5);
                                    faceCollection[i_faces].Text = namesArray[i_faces];
                                    faceCollection[i_faces].Visibility = Windows.UI.Xaml.Visibility.Visible;
                                    /*
                                    if ((((x - 320) * 2 + 320) / 2 > 0) && (((x - 320) * 2 + 320) / 2 < 300) && (((y - 240) * 2 + 240 + a * 2) / 2 > -a) && (((y - 240) * 2 + 240 + a * 2) / 2 < 230))
                                    {
                                        faceCollection[i_faces].Visibility = Windows.UI.Xaml.Visibility.Visible;
                                    }
                                    */
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
            /*
            if ((yPos1 > 200) || ((yPos1 < 50) && (xPos1 > 230)))
            {
                e.Handled = true;
                return;
            }
            */
            //posScaleFactor = 1.0;
            if (SwipeButton.IsChecked == true)
            {
                if ((e.Pointer.PointerDeviceType == Windows.Devices.Input.PointerDeviceType.Touch) || (pointerIsPressed == true))
                {
                    joyXStartPos = e.GetCurrentPoint(CamCanvas).Position.X;
                    joyYStartPos = e.GetCurrentPoint(CamCanvas).Position.Y;
                    Canvas.SetLeft(SoftJoy, joyXStartPos - SoftJoy.Width / 2);
                    Canvas.SetTop(SoftJoy, joyYStartPos - SoftJoy.Width / 2);
                    Canvas.SetLeft(HorizontalLine, joyXStartPos);
                    Canvas.SetTop(VerticalLine, joyYStartPos);
                }
            }
            e.Handled = true;
        }

        private void CanvasPointerPressed(object sender, PointerRoutedEventArgs e)
        {
            //Debug.WriteLine("kaki");
            xPos1 = Convert.ToInt16(e.GetCurrentPoint(CamCanvas).Position.X);
            yPos1 = Convert.ToInt16(e.GetCurrentPoint(CamCanvas).Position.Y);
            /*
            if ((yPos1 > 200) || ((yPos1 < 50) && (xPos1 > 230)))
            {
                e.Handled = true;
                return;
            }
            */
            if (SwipeButton.IsChecked == true)
            {
                joyXStartPos = e.GetCurrentPoint(CamCanvas).Position.X;
                joyYStartPos = e.GetCurrentPoint(CamCanvas).Position.Y;
                Canvas.SetLeft(SoftJoy, joyXStartPos - SoftJoy.Width / 2);
                Canvas.SetTop(SoftJoy, joyYStartPos - SoftJoy.Width / 2);
                Canvas.SetLeft(HorizontalLine, joyXStartPos);
                Canvas.SetTop(VerticalLine, joyYStartPos);
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
            /*
            if ((y_temp > 200) || ((y_temp < 50) && (x_temp > 230)))
            {
                e.Handled = true;
                return;
            }
            */
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

                    S1.Value = SaturateInteger(Convert.ToInt16(S1.Value + xDelta * 3), 0, 1000);
                    S2.Value = SaturateInteger(Convert.ToInt16(S2.Value + yDelta * 3), 0, 1000);
                }
            }
            else
            {
                if ((e.Pointer.PointerDeviceType == Windows.Devices.Input.PointerDeviceType.Touch) || (pointerIsPressed == true))
                {
                    joyXCurrentPos = x_temp;
                    joyYCurrentPos = y_temp;
                    Canvas.SetLeft(SoftJoy, joyXCurrentPos - SoftJoy.Width / 2);
                    Canvas.SetTop(SoftJoy, joyYCurrentPos - SoftJoy.Width / 2);
                    S1.Value = SaturateInteger(Convert.ToInt16(S1.Value + (joyXCurrentPos - joyXStartPos) * 0.1), 0, 1000);
                    S2.Value = SaturateInteger(Convert.ToInt16(S2.Value + (joyYCurrentPos - joyYStartPos) * 0.1), 0, 1000);
                }
            }

            e.Handled = true;
        }

        private void CanvasPointerReleased(object sender, PointerRoutedEventArgs e)
        {
            Debug.WriteLine("kaki");
            pointerIsPressed = false;
            if (SwipeButton.IsChecked == true)
            {
                Canvas.SetLeft(SoftJoy, CamCanvas.Width / 2 - SoftJoy.Width / 2);
                Canvas.SetTop(SoftJoy, CamCanvas.Height / 2 - SoftJoy.Width / 2);
                Canvas.SetLeft(HorizontalLine, CamCanvas.Width / 2);
                Canvas.SetTop(VerticalLine, CamCanvas.Height / 2);
            }
            e.Handled = true;
        }

        private void CanvasPointerExited(object sender, PointerRoutedEventArgs e)
        {
            if (SwipeButton.IsChecked == true)
            {
                Canvas.SetLeft(SoftJoy, CamCanvas.Width / 2 - SoftJoy.Width / 2);
                Canvas.SetTop(SoftJoy, CamCanvas.Height / 2 - SoftJoy.Width / 2);
                Canvas.SetLeft(HorizontalLine, CamCanvas.Width / 2);
                Canvas.SetTop(VerticalLine, CamCanvas.Height / 2);
            }
            e.Handled = true;
        }

        public int SaturateInteger(int val, int min, int max)
        {
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

        private void LeftDriveLost(object sender, PointerRoutedEventArgs e)
        {
            //LeftDrive.Value = 0;
            e.Handled = true;
        }

        private void RightDriveLost(object sender, PointerRoutedEventArgs e)
        {
            //RightDrive.Value = 0;
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
                SoftJoy.Visibility = Windows.UI.Xaml.Visibility.Visible;
                HorizontalLine.Visibility = Windows.UI.Xaml.Visibility.Visible;
                VerticalLine.Visibility = Windows.UI.Xaml.Visibility.Visible;
                SwipeButton.Content = "VIRTUAL JOY";
            }
            else
            {
                SoftJoy.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
                HorizontalLine.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
                VerticalLine.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
                SwipeButton.Content = "SWIPE";
            }
            PopUpCanvas.Visibility = Windows.UI.Xaml.Visibility.Collapsed;
        }
    }
}
