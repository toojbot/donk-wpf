using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
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
using Windows.Devices.Bluetooth;
using Windows.Devices.Bluetooth.Advertisement;
using Windows.Devices.Enumeration;
using Windows.Devices.Bluetooth.GenericAttributeProfile;
using Windows.Storage.Streams;
using System.Threading.Tasks;
using System.Windows.Threading;
using LiveChartsCore;
using LiveChartsCore.SkiaSharpView;
using LiveChartsCore.SkiaSharpView.Painting;
using SkiaSharp;
using System.ComponentModel;
using System.IO;
using NLog;

namespace ESP32BLE
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        private BluetoothLEAdvertisementWatcher watcher;
        private ObservableCollection<BleDeviceInfo> devices;
        private bool isScanning;
        private BluetoothLEDevice? connectedDevice;
        private GattCharacteristic? notifyCharacteristic;
        private bool isConnected = false;
        private DispatcherTimer connectionCheckTimer;
        private const int RECONNECT_INTERVAL_MS = 2000; // Check every 2 seconds
        private const int MAX_POINTS = 300; // Number of points to show in chart
        private Queue<double> soundDataPoints;
        private Queue<double> temp1DataPoints;
        private Queue<double> temp2DataPoints;
        private LineSeries<double> soundSeries;
        private LineSeries<double> temp1Series;
        private LineSeries<double> temp2Series;
        private List<(DateTime timestamp, double sound, double temp1, double temp2)> dataBuffer = new();
        private readonly object bufferLock = new();
        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        public event PropertyChangedEventHandler? PropertyChanged;

        public ObservableCollection<ISeries> Series 
        { 
            get => new ObservableCollection<ISeries> { soundSeries };
            set
            {
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(Series)));
            }
        }

        public ObservableCollection<ISeries> SoundSeries { get; set; }
        public ObservableCollection<ISeries> Temp1Series { get; set; }
        public ObservableCollection<ISeries> Temp2Series { get; set; }

        public bool IsConnected
        {
            get => isConnected;
            set
            {
                isConnected = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(IsConnected)));
            }
        }

        public List<Axis> XAxes { get; set; } = new List<Axis>
        {
            new Axis
            {
                NamePaint = new SolidColorPaint(SKColors.Black),
                LabelsPaint = new SolidColorPaint(SKColors.Black),
                TextSize = 12
            }
        };

        public List<Axis> SoundYAxes { get; set; } = new List<Axis>
        {
            new Axis
            {
                Name = "Sound",
                NamePaint = new SolidColorPaint(SKColors.Black),
                LabelsPaint = new SolidColorPaint(SKColors.Black),
                TextSize = 12,
                MinLimit = 0,
                MaxLimit = 4095
            }
        };

        public List<Axis> Temp1YAxes { get; set; } = new List<Axis>
        {
            new Axis
            {
                Name = "Temp 1",
                NamePaint = new SolidColorPaint(SKColors.Black),
                LabelsPaint = new SolidColorPaint(SKColors.Black),
                TextSize = 12,
                MinLimit = 0,
                MaxLimit = 4095
            }
        };

        public List<Axis> Temp2YAxes { get; set; } = new List<Axis>
        {
            new Axis
            {
                Name = "Temp 2",
                NamePaint = new SolidColorPaint(SKColors.Black),
                LabelsPaint = new SolidColorPaint(SKColors.Black),
                TextSize = 12,
                MinLimit = 0,
                MaxLimit = 4095
            }
        };

        public DrawMarginFrame DrawMarginFrame => new()
        {
            Fill = new SolidColorPaint(SKColors.AliceBlue),
            Stroke = new SolidColorPaint(SKColors.LightGray) { StrokeThickness = 2 }
        };

        public MainWindow()
        {
            try
            {
                // Create Logs directory if it doesn't exist
                Directory.CreateDirectory(System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Logs"));
                
                InitializeComponent();
                
                // Set default target address
                txtTargetAddress.Text = "14141152650070";
                
                InitializeBleWatcher();
                InitializeConnectionTimer();
                InitializeChart();
                
                devices = new ObservableCollection<BleDeviceInfo>();
                lvDevices.ItemsSource = devices;
                DataContext = this;

                Logger.Info("Application started successfully");
                Logger.Debug("Initialization completed");
            }
            catch (Exception ex)
            {
                Logger.Error(ex, "Error during application startup");
                MessageBox.Show($"Error during startup: {ex.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void InitializeBleWatcher()
        {
            try
            {
                Logger.Debug("Initializing BLE watcher");
                watcher = new BluetoothLEAdvertisementWatcher
                {
                    ScanningMode = BluetoothLEScanningMode.Active
                };

                watcher.Received += Watcher_Received;
                watcher.Stopped += Watcher_Stopped;

                // Start scanning automatically
                watcher.Start();
                isScanning = true;
                btnScan.Content = "Stop Scanning";
                txtStatus.Text = "Scanning for devices...";
                Logger.Info("BLE watcher started automatically");
            }
            catch (Exception ex)
            {
                Logger.Error(ex, "Failed to initialize BLE watcher");
                MessageBox.Show($"Failed to initialize Bluetooth: {ex.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void InitializeConnectionTimer()
        {
            connectionCheckTimer = new DispatcherTimer();
            connectionCheckTimer.Interval = TimeSpan.FromMilliseconds(RECONNECT_INTERVAL_MS);
            connectionCheckTimer.Tick += ConnectionCheckTimer_Tick;
        }

        private void InitializeChart()
        {
            // Initialize sound chart
            soundDataPoints = new Queue<double>(MAX_POINTS);
            soundSeries = new LineSeries<double>
            {
                Values = new ObservableCollection<double>(),
                Fill = null,
                GeometrySize = 0,
                LineSmoothness = 0.2,
                Stroke = new SolidColorPaint(SKColors.Blue) { StrokeThickness = 2 },
                Name = "Sound"
            };

            // Initialize temperature 1 chart
            temp1DataPoints = new Queue<double>(MAX_POINTS);
            temp1Series = new LineSeries<double>
            {
                Values = new ObservableCollection<double>(),
                Fill = null,
                GeometrySize = 0,
                LineSmoothness = 0.2,
                Stroke = new SolidColorPaint(SKColors.Red) { StrokeThickness = 2 },
                Name = "Temperature 1"
            };

            // Initialize temperature 2 chart
            temp2DataPoints = new Queue<double>(MAX_POINTS);
            temp2Series = new LineSeries<double>
            {
                Values = new ObservableCollection<double>(),
                Fill = null,
                GeometrySize = 0,
                LineSmoothness = 0.2,
                Stroke = new SolidColorPaint(SKColors.Green) { StrokeThickness = 2 },
                Name = "Temperature 2"
            };

            // Initialize with zero values
            for (int i = 0; i < MAX_POINTS; i++)
            {
                soundDataPoints.Enqueue(0);
                temp1DataPoints.Enqueue(0);
                temp2DataPoints.Enqueue(0);
            }

            soundSeries.Values = new ObservableCollection<double>(soundDataPoints);
            temp1Series.Values = new ObservableCollection<double>(temp1DataPoints);
            temp2Series.Values = new ObservableCollection<double>(temp2DataPoints);

            SoundSeries = new ObservableCollection<ISeries> { soundSeries };
            Temp1Series = new ObservableCollection<ISeries> { temp1Series };
            Temp2Series = new ObservableCollection<ISeries> { temp2Series };
        }

        private void UpdateChart(double soundValue, double temp1Value, double temp2Value)
        {
            // Update sound chart
            soundDataPoints.Enqueue(soundValue);
            soundDataPoints.Dequeue();
            soundSeries.Values = new ObservableCollection<double>(soundDataPoints);

            // Update temperature 1 chart
            temp1DataPoints.Enqueue(temp1Value);
            temp1DataPoints.Dequeue();
            temp1Series.Values = new ObservableCollection<double>(temp1DataPoints);

            // Update temperature 2 chart
            temp2DataPoints.Enqueue(temp2Value);
            temp2DataPoints.Dequeue();
            temp2Series.Values = new ObservableCollection<double>(temp2DataPoints);

            // Notify property changes
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(SoundSeries)));
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(Temp1Series)));
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(Temp2Series)));
        }

        private async void Watcher_Received(BluetoothLEAdvertisementWatcher sender, BluetoothLEAdvertisementReceivedEventArgs args)
        {
            await Dispatcher.InvokeAsync(() =>
            {
                try
                {
                    var existingDevice = devices.FirstOrDefault(d => d.Address == args.BluetoothAddress);
                    string deviceName = string.IsNullOrEmpty(args.Advertisement.LocalName) ? 
                                      "Unknown Device" : args.Advertisement.LocalName;

                    // Check if this is our target device
                    string targetAddress = txtTargetAddress.Text.Trim();
                    bool isTargetDevice = !string.IsNullOrEmpty(targetAddress) && 
                                        args.BluetoothAddress.ToString().EndsWith(targetAddress, StringComparison.OrdinalIgnoreCase);

                    if (existingDevice == null)
                    {
                        Logger.Debug($"New device found - Name: {deviceName}, Address: {args.BluetoothAddress}, RSSI: {args.RawSignalStrengthInDBm}");
                        var newDevice = new BleDeviceInfo
                        {
                            Name = deviceName,
                            Address = args.BluetoothAddress,
                            Rssi = args.RawSignalStrengthInDBm
                        };
                        devices.Add(newDevice);

                        // If this is our target device, select it automatically
                        if (isTargetDevice)
                        {
                            lvDevices.SelectedItem = newDevice;
                            txtStatus.Text = "Target device found!";
                            Logger.Info($"Target device found: {deviceName} ({args.BluetoothAddress})");
                        }
                    }
                    else
                    {
                        if (existingDevice.Rssi != args.RawSignalStrengthInDBm)
                        {
                            Logger.Debug($"Device updated - Name: {deviceName}, Address: {args.BluetoothAddress}, RSSI: {args.RawSignalStrengthInDBm}");
                        }
                        existingDevice.Rssi = args.RawSignalStrengthInDBm;
                        existingDevice.Name = string.IsNullOrEmpty(args.Advertisement.LocalName) 
                            ? existingDevice.Name 
                            : args.Advertisement.LocalName;

                        // If this is our target device and it's not selected, select it
                        if (isTargetDevice && lvDevices.SelectedItem != existingDevice)
                        {
                            lvDevices.SelectedItem = existingDevice;
                            txtStatus.Text = "Target device found!";
                            Logger.Info($"Target device found: {deviceName} ({args.BluetoothAddress})");
                        }
                    }
                }
                catch (Exception ex)
                {
                    Logger.Error(ex, "Error processing received BLE device");
                }
            });
        }

        private void Watcher_Stopped(BluetoothLEAdvertisementWatcher sender, BluetoothLEAdvertisementWatcherStoppedEventArgs args)
        {
            Dispatcher.Invoke(() =>
            {
                isScanning = false;
                btnScan.Content = "Start Scanning";
                txtStatus.Text = "Scanning stopped";
            });
        }

        private void btnScan_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (!isScanning)
                {
                    Logger.Info("Starting BLE scan");
                    devices.Clear();
                    watcher.Start();
                    isScanning = true;
                    btnScan.Content = "Stop Scanning";
                    txtStatus.Text = "Scanning for devices...";
                    Logger.Debug("BLE scan started");
                }
                else
                {
                    Logger.Info("Stopping BLE scan");
                    watcher.Stop();
                    isScanning = false;
                    btnScan.Content = "Start Scanning";
                    txtStatus.Text = "Scanning stopped";
                    Logger.Debug($"BLE scan stopped. Found {devices.Count} devices");
                }
            }
            catch (Exception ex)
            {
                Logger.Error(ex, "Error toggling BLE scan");
                MessageBox.Show($"Error controlling Bluetooth scan: {ex.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private async void btnPair_Click(object sender, RoutedEventArgs e)
        {
            if (lvDevices.SelectedItem is BleDeviceInfo selectedDevice)
            {
                try
                {
                    txtStatus.Text = "Attempting to pair...";
                    var deviceInfo = await BluetoothLEDevice.FromBluetoothAddressAsync(selectedDevice.Address);
                    
                    if (deviceInfo != null)
                    {
                        // Check if device is already paired
                        if (deviceInfo.DeviceInformation.Pairing.IsPaired)
                        {
                            txtStatus.Text = "Device is already paired. Attempting to unpair and repair...";
                            
                            // Unpair first
                            await deviceInfo.DeviceInformation.Pairing.UnpairAsync();

                            var result = await deviceInfo.DeviceInformation.Pairing.PairAsync();
                            txtStatus.Text = $"Pairing result: {result.Status}";
                        }
                        else
                        {
                            var result = await deviceInfo.DeviceInformation.Pairing.PairAsync();
                            if (result.Status == DevicePairingResultStatus.Failed)
                            {
                                txtStatus.Text = $"Pairing result: Ready to connect";
                            }
                            else
                            {
                                txtStatus.Text = $"Pairing result: {result.Status}";
                            }
                        }
                    }
                }
                catch (Exception ex)
                {
                    txtStatus.Text = $"Pairing failed: {ex.Message}";
                }
            }
            else
            {
                txtStatus.Text = "Please select a device to pair";
            }
        }

        private async void btnConnect_Click(object sender, RoutedEventArgs e)
        {
            if (lvDevices.SelectedItem is BleDeviceInfo selectedDevice)
            {
                try
                {
                    txtStatus.Text = "Connecting to device...";
                    connectedDevice = await BluetoothLEDevice.FromBluetoothAddressAsync(selectedDevice.Address);

                    if (connectedDevice == null)
                    {
                        txtStatus.Text = "Failed to connect to device";
                        return;
                    }

                    await EstablishConnection();
                }
                catch (Exception ex)
                {
                    txtStatus.Text = $"Connection failed: {ex.Message}";
                    IsConnected = false;
                    connectionCheckTimer.Stop();
                }
            }
            else
            {
                txtStatus.Text = "Please select a device to connect";
            }
        }

        private async Task EstablishConnection()
        {
            var result = await connectedDevice.GetGattServicesAsync();
            if (result.Status == GattCommunicationStatus.Success)
            {
                foreach (var service in result.Services)
                {
                    var characteristicsResult = await service.GetCharacteristicsAsync();
                    if (characteristicsResult.Status == GattCommunicationStatus.Success)
                    {
                        foreach (var characteristic in characteristicsResult.Characteristics)
                        {
                            if (characteristic.CharacteristicProperties.HasFlag(GattCharacteristicProperties.Notify))
                            {
                                notifyCharacteristic = characteristic;
                                
                                var status = await characteristic.WriteClientCharacteristicConfigurationDescriptorAsync(
                                    GattClientCharacteristicConfigurationDescriptorValue.Notify);
                                
                                if (status == GattCommunicationStatus.Success)
                                {
                                    characteristic.ValueChanged += Characteristic_ValueChanged;
                                    txtStatus.Text = "Connected and listening for notifications";
                                    IsConnected = true;
                                    connectionCheckTimer.Start();
                                    return;
                                }
                            }
                        }
                    }
                }
            }
            throw new Exception("Failed to establish connection");
        }

        private async void Characteristic_ValueChanged(GattCharacteristic sender, GattValueChangedEventArgs args)
        {
            var data = new byte[args.CharacteristicValue.Length];
            DataReader.FromBuffer(args.CharacteristicValue).ReadBytes(data);
            
            var value = Encoding.ASCII.GetString(data);
            
            await Dispatcher.InvokeAsync(() =>
            {
                try
                {
                    // Parse the received data
                    string[] pairs = value.Trim().Split(',');
                    Dictionary<string, int> values = new Dictionary<string, int>();

                    foreach (string pair in pairs)
                    {
                        string[] keyValue = pair.Split(':');
                        if (keyValue.Length == 2 && int.TryParse(keyValue[1], out int parsedValue))
                        {
                            values[keyValue[0]] = parsedValue;
                        }
                    }

                    // Update charts if we have valid data
                    if (values.ContainsKey("S") && values.ContainsKey("T1") && values.ContainsKey("T2"))
                    {
                        UpdateChart(values["S"], values["T1"], values["T2"]);
                        
                        // Add to buffer with timestamp
                        lock (bufferLock)
                        {
                            dataBuffer.Add((DateTime.Now, values["S"], values["T1"], values["T2"]));
                        }
                    }
                }
                catch (Exception ex)
                {
                    Logger.Error(ex, "Error parsing received data");
                }
            });
        }

        private async void ConnectionCheckTimer_Tick(object sender, EventArgs e)
        {
            if (IsConnected && connectedDevice != null)
            {
                try
                {
                    // Check connection status
                    var result = await connectedDevice.GetGattServicesAsync();
                    if (result.Status != GattCommunicationStatus.Success)
                    {
                        await ReconnectAsync();
                    }
                }
                catch
                {
                    await ReconnectAsync();
                }
            }
        }

        private async Task ReconnectAsync()
        {
            try
            {
                txtStatus.Text = "Connection lost. Attempting to reconnect...";
                
                // Clean up old connection
                if (notifyCharacteristic != null)
                {
                    notifyCharacteristic.ValueChanged -= Characteristic_ValueChanged;
                    try
                    {
                        await notifyCharacteristic.WriteClientCharacteristicConfigurationDescriptorAsync(
                            GattClientCharacteristicConfigurationDescriptorValue.None);
                    }
                    catch { }
                }

                if (connectedDevice != null)
                {
                    var address = connectedDevice.BluetoothAddress;
                    connectedDevice.Dispose();
                    connectedDevice = null;

                    // Attempt to reconnect
                    connectedDevice = await BluetoothLEDevice.FromBluetoothAddressAsync(address);
                    if (connectedDevice != null)
                    {
                        await EstablishConnection();
                    }
                }
            }
            catch (Exception ex)
            {
                txtStatus.Text = $"Reconnection failed: {ex.Message}";
                IsConnected = false;
            }
        }

        private async Task ExportToCSV()
        {
            try
            {
                string outputDir = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "output");
                Directory.CreateDirectory(outputDir);

                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                string filename = System.IO.Path.Combine(outputDir, $"data_{timestamp}.csv");
                
                StringBuilder csv = new StringBuilder();
                csv.AppendLine("Timestamp,Sound,Temperature1,Temperature2");
                
                lock (bufferLock)
                {
                    foreach (var (time, sound, temp1, temp2) in dataBuffer)
                    {
                        csv.AppendLine($"{time:yyyy-MM-dd HH:mm:ss.fff},{sound},{temp1},{temp2}");
                    }
                }

                await File.WriteAllTextAsync(filename, csv.ToString());
                txtStatus.Text = $"Data exported to {filename}";
                
                lock (bufferLock)
                {
                    dataBuffer.Clear();
                }
            }
            catch (Exception ex)
            {
                txtStatus.Text = $"Failed to export CSV: {ex.Message}";
            }
        }

        private async void btnTerminate_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                // Export data to CSV first
                await ExportToCSV();

                connectionCheckTimer.Stop();
                IsConnected = false;
                
                if (notifyCharacteristic != null)
                {
                    notifyCharacteristic.ValueChanged -= Characteristic_ValueChanged;
                    await notifyCharacteristic.WriteClientCharacteristicConfigurationDescriptorAsync(
                        GattClientCharacteristicConfigurationDescriptorValue.None);
                }

                if (connectedDevice != null)
                {
                    if (connectedDevice.DeviceInformation.Pairing.IsPaired)
                    {
                        await connectedDevice.DeviceInformation.Pairing.UnpairAsync();
                    }
                    connectedDevice.Dispose();
                    connectedDevice = null;
                }

                txtStatus.Text += "\nConnection terminated";
                
                btnScan.IsEnabled = true;
                btnPair.IsEnabled = true;
                btnConnect.IsEnabled = true;
            }
            catch (Exception ex)
            {
                txtStatus.Text = $"Termination error: {ex.Message}";
            }
        }

        protected override async void OnClosed(EventArgs e)
        {
            connectionCheckTimer.Stop();
            try
            {
                if (notifyCharacteristic != null)
                {
                    notifyCharacteristic.ValueChanged -= Characteristic_ValueChanged;
                    await notifyCharacteristic.WriteClientCharacteristicConfigurationDescriptorAsync(
                        GattClientCharacteristicConfigurationDescriptorValue.None);
                }

                if (connectedDevice != null)
                {
                    if (connectedDevice.DeviceInformation.Pairing.IsPaired)
                    {
                        await connectedDevice.DeviceInformation.Pairing.UnpairAsync();
                    }
                    connectedDevice.Dispose();
                }
            }
            catch
            {
                // Ignore errors during shutdown
            }
            
            base.OnClosed(e);
        }

        // Add helper method to validate Bluetooth address
        private bool IsValidBluetoothAddress(string address)
        {
            // Remove any colons or hyphens
            address = address.Replace(":", "").Replace("-", "");
            
            // Check if it's a valid hexadecimal number
            return !string.IsNullOrEmpty(address) && 
                   address.All(c => "0123456789ABCDEFabcdef".Contains(c));
        }
    }

    public class BleDeviceInfo
    {
        public string Name { get; set; } = string.Empty;
        public ulong Address { get; set; }
        public short Rssi { get; set; }
    }
}