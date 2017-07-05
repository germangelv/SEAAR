package com.example.juanpablo.autitoarduino;

/**
 * Created by Juan Pablo on 1/6/2017.
 */

import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.support.v7.app.AppCompatActivity;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.TextView;

import java.util.ArrayList;


/*
Un objeto Adaptador actúa como puente entre un AdapterView y los datos de una Vista (View).
 El adaptador permite el acceso a los elementos de datos, éste también es responsable de crear
 una vista para cada elemento en la colección de datos.
Se puede decir, que los adaptadores son colecciones de datos, que asignamos a una vista para que
 ésta los muestre, por ejemplo, podemos crear un ArrayAdapter a partir de un array de string ya
 creado y con datos, y asignar este adaptador a un ListView, así, el ListView mostrará los datos del array.
 */


public class DeviceListAdapter extends ArrayAdapter<BluetoothDevice> {

    private LayoutInflater mLayoutInflater;
    private ArrayList<BluetoothDevice> mDevices;
    private int  mViewResourceId;


    /*
    al crear el arrayAdapter, tenemos que pasar tres parámetros, el contexto, un layout
    que se usará para dibujar cada item y la colección de datos.

     */
    public DeviceListAdapter(Context context, int tvResourceId, ArrayList<BluetoothDevice> devices){
        super(context, tvResourceId,devices);
        this.mDevices = devices;
        mLayoutInflater = (LayoutInflater) context.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
        mViewResourceId = tvResourceId;
    }

    public View getView(int position, View convertView, ViewGroup parent) {
        convertView = mLayoutInflater.inflate(mViewResourceId, null);

        BluetoothDevice device = mDevices.get(position);

        if (device != null) {
            TextView deviceName = (TextView) convertView.findViewById(R.id.tvDeviceName);
            TextView deviceAdress = (TextView) convertView.findViewById(R.id.tvDeviceAddress);

            if (deviceName != null) {
                deviceName.setText(device.getName());
            }
            if (deviceAdress != null) {
                deviceAdress.setText(device.getAddress());
            }
        }

        return convertView;
    }


}
