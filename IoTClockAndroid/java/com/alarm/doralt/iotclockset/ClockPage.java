package com.alarm.doralt.iotclockset;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothGatt;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.pm.ResolveInfo;
import android.net.Uri;
import android.os.Environment;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import java.io.File;
import java.util.Date;
import java.util.List;

public class ClockPage {

    static String HOURS = "";
    static String MINUTES = "";
    private BluetoothGattCharacteristic mCharacteristic;
    private BluetoothGatt               mGatt;
    protected void ClockPage() {

    }

    private String getDateTime() {
        DateFormat dateFormat = new SimpleDateFormat("yyyy/MM/dd HH:mm");
        Date date = new Date();
        return dateFormat.format(date);
    }

    protected void onActivityResult(BluetoothGatt gatt,
                                    BluetoothGattCharacteristic characteristic){
        mGatt = gatt;
        mCharacteristic = characteristic;

        String data =  getDateTime() + " "+HOURS+":"+MINUTES;
        //data = "B";
        for (int i = 0; i<data.length();i++) {
            char x = data.charAt(i);
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            mCharacteristic.setValue((int) x, BluetoothGattCharacteristic.FORMAT_UINT8, 0);
            mGatt.writeCharacteristic(mCharacteristic);
        }
    }

}


