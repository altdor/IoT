/*
 * Copyright (C) 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.alarm.doralt.iotclockset;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.ExpandableListView;
import android.widget.ScrollView;
import android.widget.SimpleExpandableListAdapter;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.UUID;

/**
 * For a given BLE device, this Activity provides the user interface to connect, display data,
 * and display GATT services and characteristics supported by the device.  The Activity
 * communicates with {@code BluetoothLeService}, which in turn interacts with the
 * Bluetooth LE API.
 */
public class DeviceControlActivity extends Activity {
    private final static String TAG = DeviceControlActivity.class.getSimpleName();

    public static final String EXTRAS_DEVICE_NAME = "DEVICE_NAME";
    public static final String EXTRAS_DEVICE_ADDRESS = "DEVICE_ADDRESS";

    private TextView mConnectionState;
    private TextView mDataField;
    private String mDeviceName;
    private String mDeviceAddress;
    private BluetoothLeService mBluetoothLeService;
    private BluetoothGattCharacteristic mWriteCharacteristic;
    private ScrollView mHourButton;
    private ScrollView mMinuteButton;
    private Button mSendButton;
    private ClockPage        mCP;

    // Code to manage Service lifecycle.
    private final ServiceConnection mServiceConnection = new ServiceConnection() {

        @Override
        public void onServiceConnected(ComponentName componentName, IBinder service) {
            mBluetoothLeService = ((BluetoothLeService.LocalBinder) service).getService();
            if (!mBluetoothLeService.initialize()) {
                Log.e(TAG, "Unable to initialize Bluetooth");
                finish();
            }
            // Automatically connects to the device upon successful start-up initialization.
            mBluetoothLeService.connect(mDeviceAddress);
        }

        @Override
        public void onServiceDisconnected(ComponentName componentName) {
            mBluetoothLeService = null;
        }
    };

    // Handles various events fired by the Service.
    // ACTION_GATT_CONNECTED: connected to a GATT server.
    // ACTION_GATT_DISCONNECTED: disconnected from a GATT server.
    // ACTION_GATT_SERVICES_DISCOVERED: discovered GATT services.
    // ACTION_DATA_AVAILABLE: received data from the device.  This can be a result of read
    //                        or notification operations.
    private final BroadcastReceiver mGattUpdateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent){
                final String action = intent.getAction();
                if (BluetoothLeService.ACTION_GATT_CONNECTED.equals(action)) {
                    updateConnectionState(R.string.connected);
                    mHourButton.setVisibility(View.VISIBLE);
                    mMinuteButton.setVisibility(View.VISIBLE);
                    mSendButton.setVisibility(View.VISIBLE);

                    invalidateOptionsMenu();
                } else if (BluetoothLeService.ACTION_GATT_DISCONNECTED.equals(action)) {
                    updateConnectionState(R.string.disconnected);
                    mHourButton.setVisibility(View.INVISIBLE);
                    mMinuteButton.setVisibility(View.INVISIBLE);
                    mSendButton.setVisibility(View.INVISIBLE);
                    invalidateOptionsMenu();
                } else if (BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED.equals(action)) {
                    // Find the write characteristic of the board's GATT service
                    initializeCharacteristic(mBluetoothLeService.getSupportedGattServices());
                    // Start writing data (phone's rotation vector)
                    mBluetoothLeService.writeCharacteristic(mWriteCharacteristic,
                            mCP);
                    ;
                } else if (BluetoothLeService.ACTION_DATA_AVAILABLE.equals(action)) {
                    displayData(intent.getStringExtra(BluetoothLeService.EXTRA_DATA));
                }
            }
    };
    private void initializeCharacteristic(List<BluetoothGattService> supportedGattServices) {
        if (supportedGattServices != null) {
            String service_uuid = null;
            String wr_uuid      = Constants.BOARD_WR;
            // Finds the appropriate service
            for (BluetoothGattService gattService : supportedGattServices) {
                service_uuid = gattService.getUuid().toString();
                if (service_uuid.equals(Constants.BOARD_SERVICES)) {
                    // Retrieves the write characteristic
                    mWriteCharacteristic = gattService.getCharacteristic(UUID.fromString(wr_uuid));
                    break;
                }
            }
        }
    }

    private void updateConnectionState(final int resourceId) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {mConnectionState.setText(resourceId);
            }
        });
    }
    public void setHour(View v){
        Button b = (Button)v;
        ClockPage.HOURS = b.getText().toString();
    }
    public void setMinutes(View v){
        Button b = (Button)v;
        ClockPage.MINUTES = b.getText().toString();
    }
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        final Intent intent = getIntent();
        mDeviceName = intent.getStringExtra(EXTRAS_DEVICE_NAME);
        mDeviceAddress = intent.getStringExtra(EXTRAS_DEVICE_ADDRESS);

        // Sets up UI references.
        ((TextView) findViewById(R.id.device_address)).setText(mDeviceAddress);
        mHourButton = findViewById(R.id.hours);
        mMinuteButton = findViewById(R.id.minutes);
        mSendButton = findViewById(R.id.setBluetooth);
        mConnectionState    = findViewById(R.id.connection_state);
        mDataField          = findViewById(R.id.data_value);

        //getActionBar().setTitle(mDeviceName);
        //getActionBar().setDisplayHomeAsUpEnabled(true);
        Intent gattServiceIntent = new Intent(this, BluetoothLeService.class);
        bindService(gattServiceIntent, mServiceConnection, BIND_AUTO_CREATE);
        mCP = new ClockPage();
    }
    protected void sendData(View v) {
        registerReceiver(mGattUpdateReceiver, makeGattUpdateIntentFilter());
        if (mBluetoothLeService != null) {
            final boolean result = mBluetoothLeService.connect(mDeviceAddress);
            Log.d(TAG, "Connect request result=" + result);
        }
        //updateConnectionState(R.string.connected);
        initializeCharacteristic(mBluetoothLeService.getSupportedGattServices());
        mBluetoothLeService.writeCharacteristic(mWriteCharacteristic, mCP);
    }
    private static IntentFilter makeGattUpdateIntentFilter() {
        final IntentFilter intentFilter = new IntentFilter();
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_CONNECTED);
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_DISCONNECTED);
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED);
        intentFilter.addAction(BluetoothLeService.ACTION_DATA_AVAILABLE);
        return intentFilter;
    }


    @Override
    protected void onDestroy() {
        super.onDestroy();
        unbindService(mServiceConnection);
        mBluetoothLeService = null;
    }



    private void displayData(String data) {
        if (data != null) {
            mDataField.setText(data);
        }
    }



}
