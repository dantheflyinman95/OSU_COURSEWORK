package com.company;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanFilter;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.content.SharedPreferences;
import android.os.Handler;
import android.preference.PreferenceManager;
import android.util.Log;

import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

/*
 * BleUART
 *
 * Overview: 
 * This class is used to create and maintain a GATT
 * connection with a BLE device. The UART GATT service is used
 * for communication with external BLE devices.
 */
public class BleUART extends BluetoothGattCallback {
    private Context context;
    private BluetoothAdapter mBluetoothAdapter;
    private BluetoothLeScanner mLEScanner;
    private ScanSettings mScanSettings;
    private List<ScanFilter> filters;
    private BluetoothGatt mGatt;
    private BluetoothGattCharacteristic tx;
    private Handler mHandler;
    private Handler refreshConnectionHandler;
    private Handler sendSpeedHandler;
    private boolean btEnabled;  // flag to indicate BLE is available on device
    private boolean writeInProgress; // flag to indicate a write is currently in progress
    private Callback listener;
    private String bleOutput;
    private static final long SCAN_PERIOD = 10000;  // Stops scanning after 10 seconds.

    // UUIDs for UART service and associated characteristics.
    private static UUID UART_UUID = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    private static UUID TX_UUID   = UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");

    public interface Callback {
        void onConnected(boolean connected);
    }

    /*
     * BleUART(Context context)
     *
     * Input(s): Context context
     *
     * Post-Condition(s): 
     * BleUART class is instantiated and
     * initialized, and the Bluetooth adapter setup.
     *
     * Overview: 
     * Used to initialize the BleUART class when
     * instantiated.
     */
    public BleUART(Context context) {
        super();
        this.context = context;
        this.mHandler = new Handler();
        this.refreshConnectionHandler = new Handler();
        this.sendSpeedHandler = new Handler();
        this.btEnabled = false;
        BluetoothManager mBluetoothManager = (BluetoothManager) context.getSystemService(Context.BLUETOOTH_SERVICE);
        this.mBluetoothAdapter = mBluetoothManager.getAdapter();
        this.mGatt = null;
        this.tx = null;
        this.listener = null;
    }

    /*
     * setListener(Callback listener)
     *
     * Input(s): Callback listener
     * 
     * Post-Condition(s): 
     * Listener callback initialized.
     *
     * Overview:
     * Initializes a listener callback that is used
     * to flag when a connection has been made with a BLE device.
     */
    public void setListener(Callback listener) {this.listener = listener;}
    
    /*
     * adapter()
     * 
     * Input(s): N/A
     * 
     * Post-Condition(s): mBluetoothAdapter object returned.
     *
     * Overview: Used to access the Bluetooth adapter in external classes.
     */
    public BluetoothAdapter adapter() {return mBluetoothAdapter;}
    
    /*
     * setEnabled(boolean enable)
     *
     * Input(s): boolean enable
     *
     * Post-Condition(s): 
     * btEnabled is updated to flag if BLE
     * is available on the Android device, and if the user has granted
     * the app access to the device's Bluetooth adapter.
     *
     * Overview:
     * Used to turn BLE features on and off depending on
     * the availability of BLE on the Android device, and if the user
     * has granted the app access to the device's Bluetooth adapter.
     */
    public void setEnabled(boolean enable) {btEnabled = enable;}  // turn on bluetooth features
    
    /*
     * isEnabled()
     *
     * Input(s): N/A
     *
     * Post-Condition(s): 
     * btEnabled flag is returned.
     *
     * Overview: 
     * Allows external classes to verify if BLE
     * features have been enabled.
     */
    public boolean isEnabled() {return btEnabled;}
    
    /*
     * initialize()
     *
     * Input(s): N/A
     *
     * Post-Condition(s): 
     * If there is an existing mGATT device connection 
     * it is first closed. Then, if possible, a new mGATT connection
     * with the BLE device specified by the "ble_device_name"
     * shared preference is made.
     *
     * Overview: 
     * Closes previous mGATT connection if it exists,
     * then creates a new mGatt connection with the desired BLE device.
     */
    public void initialize() {
        disconnect();
        mLEScanner = mBluetoothAdapter.getBluetoothLeScanner();
        refreshConnectionHandler.post(refreshConnection);
    }
    
    /*
     * isConnected()
     *
     * Input(s): N/A
     *
     * Post-Condition(s): 
     * Status of GATT UART service connection
     * to BLE device, tx, is returned.
     *
     * Overview: 
     * If there is an mGATT UART service connection to a BLE
     * device, and the TX characteristic has been initialized, then
     * this method will flag that a BLE device is connected.
     */
    public boolean isConnected() {return (tx != null);}

    /*
     * refreshConnection
     *
     * Input(s): N/A
     *
     * Post-Condition(s):
     * If a GATT connection exists with a BLE device
     * that doesn't match the BLE device specified by the "ble_device_name"
     * shared preference, then mGATT is closed, and a new one
     * created with the correct device. If mGATT is initially null,
     * then a new GATT connection is made.
     *
     * Overview: 
     * This method will attempt to create a GATT connection
     * with the desired BLE device.
     */
    Runnable refreshConnection = new Runnable() {
        @Override
        public void run() {
            SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(context);
            String my_ble_device = sharedPref.getString("ble_device_name", "OCR Display");  // if key not set, use default value of true

            if (mGatt != null) {
                BluetoothDevice device = mGatt.getDevice();

                if (!my_ble_device.equals(device.getName())) {
                    mGatt.disconnect();
                    mGatt.close();
                    mGatt = null;
                    tx = null;
                }
            }

            if (mGatt == null && btEnabled) {
                mScanSettings = new ScanSettings.Builder()
                        .setScanMode(ScanSettings.SCAN_MODE_BALANCED)
                        .build();
                ScanFilter mScanFilter = new ScanFilter.Builder()
                        .setDeviceName(my_ble_device)
                        .build();
                filters = new ArrayList<>();
                filters.add(mScanFilter);

                scanLeDevice(true);
            }

            refreshConnectionHandler.removeCallbacks(refreshConnection);
            refreshConnectionHandler.postDelayed(this, 5000);  //recheck BLE device connection every 5 seconds
        }
    };

    /*
     * scanLeDevice(final boolean enable)
     *
     * Input(s): boolean enable
     *
     * Post-Condition(s): 
     * If enable is true, then mLEScanner will scan
     * for the device specified by the device name stored in the
     * "filters" input for the number of ms defined by "SCAN_PERIOD".
     * If enable is false, then mLEScanner will stop scanning if it 
     * is currently scanning.
     *
     * Overview: 
     * This method will scan for the desired BLE device that 
     * is specified by the "ble_device_name" shared preference.
     */
    public void scanLeDevice(final boolean enable) {
        if (enable) {
            mHandler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    mLEScanner.stopScan(mScanCallback);

                }
            }, SCAN_PERIOD);

            mLEScanner.startScan(filters, mScanSettings, mScanCallback);
        } else {
            mLEScanner.stopScan(mScanCallback);
        }
    }

    /*
     * ScanCallback()
     *
     * Input(s): N/A
     *
     * Post-Condition(s): 
     * If a BLE device is found during a scan by
     * mLEScanner, then a GATT connection is attempted with the
     * device. Otherwise, if there is an error during scanning,
     * then an error code is logged under "Scan Failed".
     *
     * Overview: 
     * This method is called when a scan by mLEScanner finds
     * a device that has a name matching the "ble_device_name" 
     * shared preference. This method will call the connectToDevice()
     * method to attempt a connection with the found BLE device.
     */
    private ScanCallback mScanCallback = new ScanCallback() {
        @Override
        public void onScanResult(int callbackType, ScanResult result) {
            Log.i("callbackType", String.valueOf(callbackType));
            Log.i("result", result.toString());
            BluetoothDevice btDevice = result.getDevice();
            Log.i("DEVICE", btDevice.getName());
            connectToDevice(btDevice);
        }

        @Override
        public void onScanFailed(int errorCode) {
            Log.e("Scan Failed", "Error Code: " + errorCode);
        }
    };

    /*
     * connectToDevice(BluetoothDevice device)
     *
     * Input(s): BluetoothDevice device
     *
     * Post-Condition(s): 
     * If no GATT connection already exists,
     * then the device scanner, mLEScanner, is stopped, and a GATT
     * connection is attempted with the BLE device. This connection is
     * stored in mGATT.
     *
     * Overview: This method will attempt a connection with the BLE device.
     */
    private void connectToDevice(BluetoothDevice device) {
        if (mGatt == null) {
            scanLeDevice(false);// will stop after first device detection
            mGatt = device.connectGatt(context, true, gattCallback);
        }
    }

    /*
     * disconnect()
     *
     * Input(s): N/A
     *
     * Post-Condition(s): 
     * If a GATT connection, mGatt already exists,
     * then the device is first disconnected, the GATT connection closed,
     * and then the mGatt, and tx objects cleared.
     *
     * Overview: 
     * This method will disconnect and close the GATT connection
     * with the BLE device if one exists.
     */
    public void disconnect() {
        if (mGatt != null) {
            mGatt.disconnect();
            mGatt.close();
            mGatt = null;
            tx = null;
        }
    }

    /*
     * BluetoothGattCallback()
     *
     * Input(s): N/A
     *
     * Post-Condition(s): 
     * If a GATT connection is created with the
     * BLE device, then the discoverServices() method will be run to
     * connect to the BLE device's UART service's TX characteristic.
     * tx will store the UART connection.
     *
     * Overview: 
     * This method is used to establish a GATT UART TX
     * connection with the BLE device.
     */
    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {
        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
            Log.i("onConnectionStateChange", "Status: " + status);
            switch (newState) {
                case BluetoothProfile.STATE_CONNECTED:
                    Log.i("gattCallback", "STATE_CONNECTED");
                    gatt.discoverServices();
                    break;
                case BluetoothProfile.STATE_DISCONNECTED:
                    Log.e("gattCallback", "STATE_DISCONNECTED");
                    if (mGatt != null) mGatt.connect();
                    break;
                default:
                    Log.e("gattCallback", "STATE_OTHER");
            }
        }

        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            // Save reference to each UART characteristic.
            tx = gatt.getService(UART_UUID).getCharacteristic(TX_UUID);

            // notify that connection has been made
            listener.onConnected(true);
        }

        @Override
        public void onCharacteristicWrite(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, int status) {
            super.onCharacteristicWrite(gatt, characteristic, status);

            if (status == BluetoothGatt.GATT_SUCCESS) {
                Log.d("gattCallback","Characteristic write successful");
                writeInProgress = false;
            }
        }
    };

    /*
     * send(byte[] data)
     *
     * Input(s): byte[] data
     *
     * Post-Condition(s): 
     * If tx has been initialized with the GATT
     * UART TX characteristic, then the byte data is sent to the BLE
     * device by writing to the mGatt UART service's tx characteristic.
     *
     * Overview: Sends byte data to the BLE device using the GATT UART service.
     */
    private int transmit(byte[] data) {
        if (tx == null || data == null || data.length == 0) {
            Log.i("UART", "tx null");
            // Do nothing if there is no connection or message to send.
            return 0;
        }
        // Update TX characteristic value.  Note the setValue overload that takes a byte array must be used.
        tx.setValue(data);
        writeInProgress = true; // Set the write in progress flag
        mGatt.writeCharacteristic(tx);
        // TODO: determine why the onCharacteristicWrite callback never sets writeInProgress to false, or add timeout
        //while (writeInProgress); // Wait for the flag to clear in onCharacteristicWrite
        Log.i("UART", "SENT MSG");
        return 1;
    }
    
    /*
     * send(String data)
     *
     * Input(s): String data
     *
     * Post-Condition(s): 
     * The String data input is parsed into bytes
     * and sent to the send(byte[] data) function for transmission.
     *
     * Overview: 
     * This method parses an input data string into bytes
     * for transmission to a BLE device.
     */
    private int convertStrForOutput(String data) {
        if (data != null && !data.isEmpty()) {
            Log.i("UART", data);
            return transmit(data.getBytes(Charset.forName("UTF-8")));
        }
        else {return 0;}
    }

    public void send(String outputStr) {
        bleOutput = outputStr;
        sendSpeedHandler.removeCallbacks(sendSpeed);
        sendSpeedHandler.post(sendSpeed);  //output detected speed over BLE
    }

    Runnable sendSpeed = new Runnable() {
        @Override
        public void run() {
            convertStrForOutput(bleOutput);
            sendSpeedHandler.postDelayed(this, 2000);  //resend detected speed every 2 sec
        }
    };
}
