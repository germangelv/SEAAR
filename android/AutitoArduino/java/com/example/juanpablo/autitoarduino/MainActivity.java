package com.example.juanpablo.autitoarduino;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Build;
import android.os.Bundle;
import android.support.v4.content.LocalBroadcastManager;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ListView;
import android.widget.TextView;

import java.io.IOException;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.UUID;



/**
 * Created by Juan Pablo on 1/6/2017.
 */

public class MainActivity extends AppCompatActivity implements AdapterView.OnItemClickListener, SensorEventListener{

    //AdapterView.OnItemClickListener nos permite crear el metodo onitemclick para el adapterview
    //y podes crear el enlace con el dispositivo que seleccionamos (nombre, address) de la lista
    private static final String TAG = "MainActivity";

    BluetoothAdapter mBluetoothAdapter; //Se crea el adaptador BT, el cual representa el adaptador BT local
    ImageButton btnEnableDisable_Discoverable; //boton para habilitar o desahabilitar que el BTadapter pueda ser visible

    /********************ESTOS SON PARA EL MANEJO*****************************************************/
    BluetoothConnectionService mBluetoothConnection; //Se crea un nuevo objeto BTconnectionService
    ImageButton btnStartConnection; //BTCS
    //Button btnSend; //BTCS
    ImageButton btnStartEngine;
    int encenderFlag = 0;
/*
    ImageButton btnAdelante;
    ImageButton btnIzquierda;
    ImageButton btnDerecha;
    ImageButton btnAtras;
*/
    TextView incomingMessages; //mensajes entrantes que se ven en el chatbox
    StringBuilder messages;

    //EditText etSend;


    //El UUID es un identificador unico universal, se usa para identificar de manera única el servicio Bluetooth de tu aplicación.
    //debe ser el mismo utilizado en BT connectionService
    //private static final UUID MY_UUID_INSECURE = UUID.fromString("8ce255c0-200a-11e0-ac64-0800200c9a66");
    private static final UUID MY_UUID_INSECURE = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");



    BluetoothDevice mBTDevice; //dispositivo BT remoto

    /**************************************************************************************************/

    /*Declaraciones para EL ACELEROMETRO */

    TextView textoAcelerometro;
    SensorManager sensorManager;
    private Sensor acelerometro;
    boolean estaEmparejado = false ;
    int flag = 0;

    /*Declaraciones para EL Sensor de luminosidad */
    private Sensor sensorLuminosidad;
    TextView textoLuminosidad;

    /*Declaraciones sensor de aproximidad*/

    private Sensor sensorAproximidad;
    TextView textoProximidad;

    /***********************************************/
    /***************LIENZO**************************/

    Lienzo lienzo;

    /***************************************************/

    public ArrayList<BluetoothDevice> mBTDevices = new ArrayList<>(); //este ArrayList va a tener todos los dipositivos que descubrimos
    public DeviceListAdapter mDeviceListAdapter;//la lista de los dispositivos BT encontrados
    ListView lvNewDevices; //la vista de la lista de los dispositivos encontrados

    /*
     Se crea una Broadcast receiver para activacion o desactivacion del BT, ACTION_STATE_CHANGED
     Atrapa el intent que lanza el metodo enableDisableBT()
    */
    private final BroadcastReceiver mBroadcastReceiver1 = new BroadcastReceiver() {
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            //Se atrapa el pedido de cambio de estado que se lanzo en el metodo enableDisableBT()
            if ( action.equals( mBluetoothAdapter.ACTION_STATE_CHANGED ) ) {
                final int state = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, mBluetoothAdapter.ERROR);

                //atrapa cada cambio de estado del BT
                switch(state){
                    case BluetoothAdapter.STATE_OFF:
                        Log.d(TAG, "onReceive: ESTADO APAGADO");
                        break;
                    case BluetoothAdapter.STATE_TURNING_OFF:
                        Log.d(TAG, "mBroadcastReceiver1: ESTADO APAGANDOSE");
                        break;
                    case BluetoothAdapter.STATE_ON:
                        Log.d(TAG, "mBroadcastReceiver1: ESTADO ENCENDIDO");
                        break;
                    case BluetoothAdapter.STATE_TURNING_ON:
                        Log.d(TAG, "mBroadcastReceiver1: ESTADO ENCENDIENDOSE");
                        break;
                }
            }
        }
    };

    /**
     * Broadcast Receiver2 para detectar cambios en el action_scan_mode
     * para que pueda ser detectado por otros dispositivos
     * ACTION_SCAN_MODE_CHANGED
     * Atrapa el intent que lanza el metodo btnEnableDisable_Discoverable()
     */
    private final BroadcastReceiver mBroadcastReceiver2 = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();

            if (action.equals(BluetoothAdapter.ACTION_SCAN_MODE_CHANGED)) {

                int mode = intent.getIntExtra(BluetoothAdapter.EXTRA_SCAN_MODE, BluetoothAdapter.ERROR);

                switch (mode) {
                    //Device is in Discoverable Mode
                    case BluetoothAdapter.SCAN_MODE_CONNECTABLE_DISCOVERABLE:
                        Log.d(TAG, "mBroadcastReceiver2: Capacidad de detección activada.");
                        break;
                    //Device not in discoverable mode
                    case BluetoothAdapter.SCAN_MODE_CONNECTABLE:
                        Log.d(TAG, "mBroadcastReceiver2: Descubrimiento inhabilitado. Capaz de recibir conexiones.");
                        break;
                    case BluetoothAdapter.SCAN_MODE_NONE:
                        Log.d(TAG, "mBroadcastReceiver2: \n" +
                                "Descubrimiento inhabilitado. No es posible recibir conexiones.");
                        break;
                    case BluetoothAdapter.STATE_CONNECTING:
                        Log.d(TAG, "mBroadcastReceiver2: Conectando....");
                        break;
                    case BluetoothAdapter.STATE_CONNECTED:
                        Log.d(TAG, "mBroadcastReceiver2: Conectado.");
                        break;
                }

            }
        }
    };




    /**
     * El boton que busca dispositivos BT (btnFindUnpairedDevices) ejecuta el metod btnDiscover()
     * el cual llama a un intent que es atrapado por el broadcast receiver 3
     */
    private BroadcastReceiver mBroadcastReceiver3 = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            Log.d(TAG, "onReceive: ACTION FOUND.");
            /*
            Con el BT Action Found, si se encuentra un dispostivo se lo coloca en la lista
             */
            if (action.equals(BluetoothDevice.ACTION_FOUND)){
                BluetoothDevice device = intent.getParcelableExtra (BluetoothDevice.EXTRA_DEVICE);
                mBTDevices.add(device);
                Log.d(TAG, "onReceive: " + device.getName() + ": " + device.getAddress());
                    /*
                       al crear el arrayAdapter, tenemos que pasar tres parámetros, el contexto, un layout
                        que se usará para dibujar cada item y la colección de datos.
                     */
                mDeviceListAdapter = new DeviceListAdapter(context, R.layout.device_adapter_view, mBTDevices);
                lvNewDevices.setAdapter(mDeviceListAdapter);
            }
        }
    };

    /**
     * Broadcast Receiver que detecta cambios en el bond state (estado de enlace con otro disp BT)
     * Atrapa el intent que se lanza cuando se detecta este cambio
     */
    private final BroadcastReceiver mBroadcastReceiver4 = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();

            //solo se hacen logs para ver como cambian de estados
            if(action.equals(BluetoothDevice.ACTION_BOND_STATE_CHANGED)){
                BluetoothDevice mDevice = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                //3 casos:
                //caso1: si ya esta enlazado
                if (mDevice.getBondState() == BluetoothDevice.BOND_BONDED){


                    Log.d(TAG, "BroadcastReceiver: VINCULADO, BOND_BONDED.");

                    mBTDevice = mDevice;

                }
                //caso2: creando el enlace

                if (mDevice.getBondState() == BluetoothDevice.BOND_BONDING) {
                    Log.d(TAG, "BroadcastReceiver: VINCULANDO, BOND_BONDING.");
                }
                //caso3: rompiendo el enlace
                if (mDevice.getBondState() == BluetoothDevice.BOND_NONE) {
                    Log.d(TAG, "BroadcastReceiver: DESVINCULADO, BOND_NONE.");
                }
            }
        }
    };


    //se cierran los broadcastReceiver cuando la aplicacion se cierra
    @Override
    protected void onDestroy() {
        Log.d(TAG, "onDestroy: called.");
        super.onDestroy();
        unregisterReceiver(mBroadcastReceiver1);
        unregisterReceiver(mBroadcastReceiver2);
        unregisterReceiver(mBroadcastReceiver3);
        unregisterReceiver(mBroadcastReceiver4);
        //mBluetoothAdapter.cancelDiscovery();
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //se inicializan los botones
        ImageButton btnONOFF = (ImageButton) findViewById(R.id.btnONOFF);
        btnEnableDisable_Discoverable = (ImageButton) findViewById(R.id.btnDiscoverable_on_off);
        lvNewDevices = (ListView) findViewById(R.id.lvNewDevices);
        mBTDevices = new ArrayList<>();


        /********************** para el acelerometro*******************/

        textoAcelerometro = (TextView)findViewById(R.id.textoAcelerometro);

        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE); //me conecto con los sensores del dispositivo
        acelerometro = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

        estaEmparejado = false;

        /********************** para el sensor luminoso **************/

        sensorLuminosidad = sensorManager.getDefaultSensor(Sensor.TYPE_LIGHT);
        textoLuminosidad = (TextView)findViewById(R.id.textoLuminosidad);

        /***************************sensor de apoximidad**************/

        sensorAproximidad = sensorManager.getDefaultSensor(Sensor.TYPE_PROXIMITY);
        textoProximidad = (TextView)findViewById(R.id.textoProximidad);



        lienzo = (Lienzo)findViewById(R.id.lienzo);


/********************** PARA la comuniciacion y botones de manejo del auto /******************************************/
        btnStartEngine = (ImageButton) findViewById(R.id.btnStartEngine);
        btnStartConnection = (ImageButton) findViewById(R.id.btnStartConnection);
        //btnSend = (Button) findViewById(R.id.btnSend);
        //etSend = (EditText) findViewById(R.id.editText);
        /*
        btnAdelante = (ImageButton) findViewById(R.id.btnAdelante);
        btnIzquierda = (ImageButton) findViewById(R.id.btnIzquierda);
        btnDerecha = (ImageButton) findViewById(R.id.btnDerecha);
        btnAtras = (ImageButton) findViewById(R.id.btnAtras);
        */
        incomingMessages = (TextView) findViewById(R.id.incomingMessage);
        messages = new StringBuilder();
        LocalBroadcastManager.getInstance(this).registerReceiver(mReceiver, new IntentFilter("incomingMessage"));

/***********************************************************************************************************/



        //Cuando el estado del enlace del adaptador BT con otro dispositivo cambia se lanza este intent q atrapa el BroadCASTreceiver 4
        IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
        registerReceiver(mBroadcastReceiver4, filter);


        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();//se obtiene el adaptador BT por default, con el metodo estatico getDefaltAdapter

        lvNewDevices.setOnItemClickListener(MainActivity.this);


        //boton que prende o apaga el BT, llama al metodo enableDisableBT
        btnONOFF.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Log.d(TAG, "onClick: habilitando/deshabilitando bluetooth.");
                enableDisableBT();
            }
        });

        //boton que comienza la conexion, llama al metodo startConnection
        btnStartConnection.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                startConnection();
            }
        });

        //boton que manda la F para que arranque el autito
        btnStartEngine.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {


                if( encenderFlag == 0  ){
                    String encender = "f";
                    byte[] bytes = encender.getBytes();
                    if( estaEmparejado == true  ){
                        mBluetoothConnection.write(bytes);
                        encenderFlag = 1;
                    }
                }else{
                    String apagar = "o";
                    byte[] bytes = apagar.getBytes();
                    if( estaEmparejado == true  ){
                        mBluetoothConnection.write(bytes);
                        encenderFlag = 0;
                    }
                }





            }
        });


/*
        //este metodo convierte lo que hay en el editText a bytes, y lo manda a la conexion a la mBluetoothConnection
        btnSend.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                byte[] bytes = etSend.getText().toString().getBytes(Charset.defaultCharset());
                if( estaEmparejado == true  ) {
                    mBluetoothConnection.write(bytes);
                    etSend.setText("");
                }
            }
        });
*/
/*

        btnAdelante.setOnClickListener(new View.OnClickListener()
        {
            @Override
            public void onClick(View view)
            {
                String adelante = "w";
                byte[] bytes = adelante.getBytes();
                if( estaEmparejado == true  ){
                    mBluetoothConnection.write(bytes);
                }

            }
        });

        btnIzquierda.setOnClickListener(new View.OnClickListener()
        {
            @Override
            public void onClick(View view)
            {
                String izquierda = "a";
                byte[] bytes = izquierda.getBytes();
                if( estaEmparejado == true  ) {
                    mBluetoothConnection.write(bytes);
                }
            }
        });

        btnDerecha.setOnClickListener(new View.OnClickListener()
        {
            @Override
            public void onClick(View view)
            {
                String derecha = "d";
                byte[] bytes = derecha.getBytes();
                if( estaEmparejado == true  ) {
                    mBluetoothConnection.write(bytes);
                }
            }
        });

        btnAtras.setOnClickListener(new View.OnClickListener()
        {
            @Override
            public void onClick(View view)
            {
                String atras = "s";
                byte[] bytes = atras.getBytes();
                if( estaEmparejado == true  ) {
                    mBluetoothConnection.write(bytes);
                }
            }
        });
*/
    } //aca termina el onCreate


    //Aca es donde se reciben los mensajes, en el text view

    BroadcastReceiver mReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String text = intent.getStringExtra("theMessage");


            // messages.append(text + "\n");
            //incomingMessages.setText(messages);
            String coordenadas[] = text.split(" ");

            if( coordenadas.length == 2 ){
                Float x = Float.parseFloat(coordenadas[0]);
                Float y = Float.parseFloat(coordenadas[1]);
                lienzo.dibujar( x , y );
            }

            incomingMessages.setText("");
            incomingMessages.append(text);


        }
    };



    //CREA UN METODO PARA LA CONEXION QUE COMIENZA
    public void startConnection(){
        startBTConnection(mBTDevice,MY_UUID_INSECURE);
    }

    /**
     * COMIENZA EL METODO DEL "chat" SERVICE
     *  El protocolo RFCOMM es un conjunto simple de protocolos de transporte
     *  es a menudo denominado emulación de puertos serie.
     *  El puerto serie de Bluetooth está basado en este protocolo.
     */
    public void startBTConnection(BluetoothDevice device, UUID uuid){
        Log.d(TAG, "startBTConnection: Inicializando RFCOM Bluetooth Connection.");

        mBluetoothConnection.startClient(device,uuid);

        Log.d(TAG, "ESTA EMPAREJADO");
        estaEmparejado = true;
    }




    //Este metodo metodo es el que prende o apaga el BT, se dispara cuando se hace click en el boton ONOFF
    public void enableDisableBT(){
        if(mBluetoothAdapter == null){
            //el telefono no tiene BT, no lo puede usar
            Log.d(TAG, "enableDisableBT: No tenes bluetooth.");
        }
        //Si el BT no esta habilitado, entonces se lo habilita
        if( !mBluetoothAdapter.isEnabled() ){
            Log.d(TAG, "enableDisableBT: habilitando BT.");
            Intent enableBTIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivity(enableBTIntent);

            //se crea un intent filter, el cual es un filtro que intercepta cambio en el estado del BT
            IntentFilter BTIntent = new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED);
            registerReceiver(mBroadcastReceiver1, BTIntent);
        }

        //si el BT esta habilitado se lo deshabilita
        if(mBluetoothAdapter.isEnabled()){
            Log.d(TAG, "enableDisableBT: deshabilitando BT.");
            mBluetoothAdapter.disable();

            IntentFilter BTIntent = new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED);
            registerReceiver(mBroadcastReceiver1, BTIntent);
        }

    }

    // este metodo es para habilitar que el dispositivo pueda ser descubierto por otros dispositivos BT
    //se activa cuando se have click en el boton btnDiscoverable_on_off
    public void btnEnableDisable_Discoverable(View view) {
        Log.d(TAG, "btnEnableDisable_Discoverable: haciendo al dispositivo descubrible por 300 segundos.");

        Intent discoverableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
        discoverableIntent.putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, 300);
        startActivity(discoverableIntent);

        //se pone el broadcast receiver para que el intentfilter intercepte los cambios
        IntentFilter intentFilter = new IntentFilter(mBluetoothAdapter.ACTION_SCAN_MODE_CHANGED);
        registerReceiver(mBroadcastReceiver2,intentFilter);

    }

    //Aca esta lo que hace el boton que busca dispositivos BT (btnFindUnpairedDevices)
    public void btnDiscover(View view) {
        Log.d(TAG, "btnDiscover: Buscando dispositivos no emparejados.");

        //Si el BT ya esa en discovering mode, si ya esta buscando dispositivos, cancela la busqueda
        if(mBluetoothAdapter.isDiscovering()){
            mBluetoothAdapter.cancelDiscovery();
            Log.d(TAG, "btnDiscover: cancelando busqueda.");

            //check BT permissions in manifest
            //si usamos cualquier version mas reciente de android que lollipop
            //se necesita un specialpermissionchek para comenzar a buscar dispositivos, sino la app no va a poder
            checkBTPermissions();

            //si estaba buscando y se apreta el boton, se cancela la busqueda y se comienza a buscar otra vez
            mBluetoothAdapter.startDiscovery();
            IntentFilter discoverDevicesIntent = new IntentFilter(BluetoothDevice.ACTION_FOUND); //hacemos un intent, que va a atrapar el broadcast receiver 3
            registerReceiver(mBroadcastReceiver3, discoverDevicesIntent);
        }

        //si no esta buscando, queremos que empiece a buscar
        if( !mBluetoothAdapter.isDiscovering()){

            checkBTPermissions();

            mBluetoothAdapter.startDiscovery();
            IntentFilter discoverDevicesIntent = new IntentFilter(BluetoothDevice.ACTION_FOUND);
            registerReceiver(mBroadcastReceiver3, discoverDevicesIntent);
        }
    }

    /**
     * Esto chequeo de permissos se usa si queres buscar dispositivos y tenes una version de android
     * mas nueva que Lollipop
     * Checkea la version de android y si es mas nueva que Lollipop hace ese chequeo de los permisos
     */
    private void checkBTPermissions() {
        if(Build.VERSION.SDK_INT > Build.VERSION_CODES.LOLLIPOP){
            int permissionCheck = this.checkSelfPermission("Manifest.permission.ACCESS_FINE_LOCATION");
            permissionCheck += this.checkSelfPermission("Manifest.permission.ACCESS_COARSE_LOCATION");
            if (permissionCheck != 0) {

                this.requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION}, 1001); //cualquier numero
            }
        }else{
            Log.d(TAG, "checkBTPermissions: No need to check permissions. SDK version < LOLLIPOP.");
        }
    }



    //el implements AdapterView.OnItemClickListener nos permite crear el metodo onitemclick para el adapterview
    //y podes crear el enlace con el dispositivo que seleccionamos (nombre, address) de la lista
    @Override
    public void onItemClick(AdapterView<?> adapterView, View view, int i, long l) {
        //primero se cancela la busqueda porque usa mucha memoria
        mBluetoothAdapter.cancelDiscovery();

        //Se obtiene
        Log.d(TAG, "onItemClick: Seleccionaste un dispositivo.");
        String deviceName = mBTDevices.get(i).getName();
        String deviceAddress = mBTDevices.get(i).getAddress();

        Log.d(TAG, "onItemClick: Nombre del dispositivo = " + deviceName);
        Log.d(TAG, "onItemClick: Direccion del dispositivo = " + deviceAddress);

        //Se crea el enlace
        //Requiere API 17+, creo que es JellyBean
        if(Build.VERSION.SDK_INT > Build.VERSION_CODES.JELLY_BEAN_MR2){
            Log.d(TAG, "Tratando de emparejar con " + deviceName);
            mBTDevices.get(i).createBond();

            mBTDevice = mBTDevices.get(i);
            mBluetoothConnection = new BluetoothConnectionService(MainActivity.this); //se instacia el BTconnectionService creado arriba de todo
        }
    }


    /* METODOS PARA EL ACELEROMETRO */

    //las dos funciones que hay que implementar si o si son onSensorChange y onAcurancyChange

    @Override
    public void onSensorChanged(SensorEvent event) {


        if ( event.sensor.getType() == Sensor.TYPE_PROXIMITY ){
            textoProximidad.setText("Prox: "+event.values[0]);

            if( estaEmparejado == true ){
                if ( event.values[0] < 5 ){
                    String pare = "p";
                    byte[] bytes = pare.getBytes();
                    mBluetoothConnection.write(bytes);
                }
            }

        }


        if( event.sensor.getType() == Sensor.TYPE_LIGHT  ){


            boolean luzEncendida = false;

            //textoLuminosidad.setText("");

            textoLuminosidad.setText("Lux: "+event.values[0]);

            if ( estaEmparejado == true ) {

                if ( event.values[0] < 40 && luzEncendida == false  ){
                    String luz = "l";
                    byte[] bytes = luz.getBytes();
                    mBluetoothConnection.write(bytes);
                    luzEncendida = true;
                }

                if ( event.values[0] > 45 && luzEncendida == true  ){
                    String noLuz = "k";
                    byte[] bytes = noLuz.getBytes();
                    mBluetoothConnection.write(bytes);
                }

            }

        }

        if( event.sensor.getType() == Sensor.TYPE_ACCELEROMETER ){

            float x,y,z;
            x = event.values[0];
            y = event.values[1];
            z = event.values[2];

            textoAcelerometro.setText("");
            textoAcelerometro.append("X: " + x  + "\n" +"Z: " + z);

            if( estaEmparejado == true ) {

                if ( z > 7 && z < 8 && flag != 1) {
                    String adelante = "w";
                    byte[] bytes = adelante.getBytes();
                    mBluetoothConnection.write(bytes);
                    flag = 1;
                }

                if ( z > 10 && flag != 2 ) {
                    String maxVel = "j";
                    byte[] bytes = maxVel.getBytes();
                    mBluetoothConnection.write(bytes);
                    flag = 2;
                }

                if ( z < -2 && flag != 3 ) {
                    String atras = "s";
                    byte[] bytes = atras.getBytes();
                    mBluetoothConnection.write(bytes);
                    flag = 3;
                }

                if ( x < -3 && flag != 4 ) {
                    String derecha = "d";
                    byte[] bytes = derecha.getBytes();
                    mBluetoothConnection.write(bytes);
                    flag = 4;

                }


                if ( x > 3 && flag != 5 ) {
                    String izquierda = "i";
                    byte[] bytes = izquierda.getBytes();
                    mBluetoothConnection.write(bytes);
                    flag = 5;
                }

                if ( x > -1 && x < 1 && flag != 6 && ( flag == 4 || flag == 5 ) ) {
                    String enderezar = "x";
                    byte[] bytes = enderezar.getBytes();
                    mBluetoothConnection.write(bytes);
                    flag = 6;
                }
            }
        }


    }





    //No lo tocamos este metodo porque actua cuando cambia la precision del sensor, no es util para nuestro caso
    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    //Se registra el listener (hay que hacerlo si o si) en el onResume y se para en el onPause
    //Se registra el listener que se ejecuta luego del onCreate pero antes de entrar a modo running
    @Override
    protected void onResume() {
        super.onResume();
        sensorManager.registerListener(this, acelerometro, SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener(this, sensorLuminosidad,SensorManager.SENSOR_DELAY_NORMAL);
    }

    //para cuando salga la app de modo running y se pause, se desregistre el listener, no se este escuchando el acelerometro
    //para que no use tanto la bateria.
    @Override
    protected void onPause() {
        super.onPause();
        sensorManager.unregisterListener(this);
    }
}
