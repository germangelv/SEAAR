package com.example.juanpablo.autitoarduino;

import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.support.v4.content.LocalBroadcastManager;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.Charset;
import java.util.UUID;


/**
 * Created by Juan Pablo on 3/6/2017.
 */

public class BluetoothConnectionService {

    private static final String TAG = "BTConnectionServ";

    int bandera = 0;

    //Se le da un nombre a la app
    private static final String appName = "MYAPP";

    //crea una variable UUID, que es una direccion que el dispositivo usa para conectarse con otro dispositivo

    /*
     * Un identificador único universal (UUID) es un formato estandarizado de 128 bits para un ID de string empleado
     * para identificar información de manera exclusiva. La ventaja de un UUID es que es suficientemente grande
     * como para que puedas seleccionar cualquiera al azar y no haya conflictos. En este caso, se usa para identificar
     * de manera única el servicio Bluetooth de tu aplicación.
     */

    //private static final UUID MY_UUID_INSECURE = UUID.fromString("8ce255c0-200a-11e0-ac64-0800200c9a66");
    private static final UUID MY_UUID_INSECURE = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");


    //crea un  bt adapter
    private final BluetoothAdapter mBluetoothAdapter; //Representa el adaptador BT local
    Context mContext;

    private AcceptThread mInsecureAcceptThread;

    private ConnectThread mConnectThread;
    private BluetoothDevice mmDevice; //Representa un adaptador BT remoto
    private UUID deviceUUID;
    ProgressDialog mProgressDialog;

    private ConnectedThread mConnectedThread;


    /* creacion del bluetooth connection service */
    public BluetoothConnectionService(Context context) {
        mContext = context;
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        start(); //cuando el BTCS es creado, se llama al acceptThread con este metodo
    }


    /**
     * Este Thread va a estar corriendo escuchando conexiones entrantes.
     * funciona como un serverside client. corre hasta que una conexion es aceptada
     * El HC-05 esta diseñado para estar parado aca, en el acceptThread, esperando que algun dispositivo se quiera conectar
     * En este caso nuestro celular.
     */
    private class AcceptThread extends Thread {

        // El server socket local
        private final BluetoothServerSocket mmServerSocket;


        public AcceptThread(){
            BluetoothServerSocket tmp = null;

            // Se crea un nuevo server socket que escucha
            try{
                tmp = mBluetoothAdapter.listenUsingInsecureRfcommWithServiceRecord(appName, MY_UUID_INSECURE);

                Log.d(TAG, "AcceptThread: Configurando el servidor usando: " + MY_UUID_INSECURE);
            }catch (IOException e){
                Log.e(TAG, "AcceptThread: IOException: " + e.getMessage() );
            }

            mmServerSocket = tmp;
        }

        //el metodo run no es necesario llamarlo, se ejecuta automaticamente cuando un accept thread es creado
        public void run(){
            Log.d(TAG, "run: AcceptThread corriendo.");

            BluetoothSocket socket = null;

            try{
                Log.d(TAG, "run: RFCOM server socket comienza.....");
                // el codigo parara aca y esperara hasta q la conexion este hecha
                // o la conexion falle
                //es un llamado bloqueante
                socket = mmServerSocket.accept();//socket al que se van conectar los dispositivos

                Log.d(TAG, "run: RFCOM server socket conexion aceptada.");

            }catch (IOException e){
                Log.e(TAG, "AcceptThread: IOException: " + e.getMessage() );
            }

            //
            if(socket != null){
                connected(socket,mmDevice);
            }

            Log.i(TAG, "END mAcceptThread ");
        }

        //este metodo cierra el servicio del socket
        public void cancel() {
            Log.d(TAG, "cancel: Cancelando AcceptThread.");
            try {
                mmServerSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "cancel: Se cierra el AcceptThread, ServerSocket fallo. " + e.getMessage() );
            }
        }

    }//finaliza el accept Thread

    /**
     * Este metodo inicia la conexion con el accept Thread
     * Los dos dispositivos van a estar con el acceptThread corriendo,
     * hasta que el connectThread de alguno de ellos arranque, en este caso nuestro cel, y agarre el socket y se conecte a el.
     * Este thread se ejecuta al intentar realizar una conexion de salida
     * con el otro dispositivo. Funciona hasta que la conexion tiene exito o falla
     */
    private class ConnectThread extends Thread {
        private BluetoothSocket mmSocket;

        //condtructor por defecto
        public ConnectThread(BluetoothDevice device, UUID uuid) {
            Log.d(TAG, "ConnectThread: iniciado.");
            mmDevice = device;
            deviceUUID = uuid;
        }

        //el metodo run no es necesario llamarlo, se ejecuta automaticamente cuando un connect thread es creado
        public void run(){
            BluetoothSocket tmp = null;
            Log.i(TAG, "RUN mConnectThread ");

            // se Obtiene un BluetoothSocket para una conexión con el dispositivo BT dado
            try {
                Log.d(TAG, "ConnectThread: Tratando de crear InsecureRfcommSocket usando UUID: " + MY_UUID_INSECURE );
                tmp = mmDevice.createRfcommSocketToServiceRecord(deviceUUID);
            } catch (IOException e) {
                Log.e(TAG, "ConnectThread: No se pudo crear un InsecureRfcommSocket " + e.getMessage());
            }

            mmSocket = tmp;// le asignas nuesto bt socket a la variable temporal

            // Cancelamos el discovery si la conexion fue hecha, ya que en el discovery mode usa mucha memoria
            //es bueno cancelarlo cada vez que ya no se lo necesite mas
            mBluetoothAdapter.cancelDiscovery();

            // trata de hacer una conexion con el BT socket

            try {
                // Esta es ubna llamada bloqueante y solo va a retonrnar
                // una conexion exitosa o una excepcion, si pasa este punto, es porq fue exitosa
                mmSocket.connect();

                Log.d(TAG, "run: ConnectThread conectado.");
            } catch (IOException e) {
                // Cierra el socket si hay una excepcion
                try {
                    mmSocket.close();
                    Log.d(TAG, "run: Socket Cerrado.");
                } catch (IOException e1) {
                    Log.e(TAG, "mConnectThread: run: No se puede cerrar la conexion en el socket " + e1.getMessage());
                }
                Log.d(TAG, "run: ConnectThread: No se puede conectar a UUID: " + MY_UUID_INSECURE );
            }

            //
            connected(mmSocket,mmDevice);
        }
        public void cancel() {
            try {
                Log.d(TAG, "cancel: Cerrando socket cliente.");
                mmSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "cancel: close() de mmSocket en Connectthread fallo. " + e.getMessage());
            }
        }
    }//finaliza el connectThread



    /**
     * Este metodo comienza el AcceptThread, asi este se inicia y se pone a esperar por una conexión.
     * Empieza una sesion en modo de escucha de servidor. LLamado por el activity onResume()
     * listening (Server) mode
     *
     */
    public synchronized void start() { //este start lo usamos para iniciar nuestro accept thread
        Log.d(TAG, "start");

        // si el threath de conexion existe lo cancelamos y creamos uno nuevo
        // si no existe empezamos uno nuevo
        if (mConnectThread != null) {
            mConnectThread.cancel();
            mConnectThread = null;
        }
        if (mInsecureAcceptThread == null) {
            mInsecureAcceptThread = new AcceptThread();
            mInsecureAcceptThread.start(); //este metodo star es nativo de la clase thread
        }
    }

    /**
     * Este metodo inicia el connectThread()
     * A continuación, ConnectThread se inicia e intenta establecer una conexión
     * con los otros dispositivos que estan parados en el AcceptThread., en este caso el HC-05
     **/

    public void startClient(BluetoothDevice device,UUID uuid){
        Log.d(TAG, "startClient: Started.");

        //initprogress dialog
        mProgressDialog = ProgressDialog.show(mContext,"Connecting Bluetooth"
                ,"Please Wait...",true);

        mConnectThread = new ConnectThread(device, uuid);
        mConnectThread.start();
    }

    /**
     EL ConnectedThread que es el responsable de mantener la conexion BT, enviando data y recibiendo
     data entrante a traves de los input/output streams respectivamente
     **/
    private class ConnectedThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;



        //condtructor por defecto
        public ConnectedThread(BluetoothSocket socket) {
            Log.d(TAG, "ConnectedThread: Iniciando.");

            mmSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            //dismiss the progressdialog when connection is established
            try{
                mProgressDialog.dismiss();
            }catch (NullPointerException e){
                e.printStackTrace();
            }


            try {
                tmpIn = mmSocket.getInputStream();
                tmpOut = mmSocket.getOutputStream();
            } catch (IOException e) {
                e.printStackTrace();
            }

            mmInStream = tmpIn;
            mmOutStream = tmpOut;


        }

        public void run(){
            byte[] buffer = new byte[1024];  // buffer store para el stream

            int bytes; // bytes retornados desde read()

            bandera = 1;

            // se queda escuchando al InputStream hasta que una excepcion ocurre
            while (true) {
                // lee del InputStream
                try {
                    bytes = mmInStream.read(buffer);
                    String incomingMessage = new String(buffer, 0, bytes);
                    Log.d(TAG, "InputStream: " + incomingMessage);

                    //
                    Intent incomingMessageIntent = new Intent("incomingMessage");
                    incomingMessageIntent.putExtra("theMessage", incomingMessage);
                    LocalBroadcastManager.getInstance(mContext).sendBroadcast(incomingMessageIntent);

                } catch (IOException e) {
                    Log.e(TAG, "write: Error leyendo Input Stream. " + e.getMessage() );
                    break;
                }
            }
        }

        //se llama a este metodo desde el main activity para enviar informacion al otro dispositivo
        public void write(byte[] bytes) {
            String text = new String(bytes, Charset.defaultCharset());
            Log.d(TAG, "write: escribiendo al outputstream: " + text);
            try {
                mmOutStream.write(bytes);
            } catch (IOException e) {
                Log.e(TAG, "write: error escribiendo output stream. " + e.getMessage() );
            }
        }

        /* este se llama desde el main activity para termianr la conexion */
        public void cancel() {
            try {
                mmSocket.close();
            } catch (IOException e) { }
        }
    }

    //este metodo es llamado desde el acceptThread y el connectThread
    private void connected(BluetoothSocket mmSocket, BluetoothDevice mmDevice) {
        Log.d(TAG, "connected: Iniciando.");

        // Inicia el thread para administrar la conexión y realizar transmisiones
        mConnectedThread = new ConnectedThread(mmSocket);
        mConnectedThread.start();
    }

    /**
     *Se escribe otro metodo write porque el anterior no podra ser accedido desde el mainactivity
     *
     * Escribe al ConnectedThread de una manera no sincronizada
     *
     * @param out The bytes to write
     * @see ConnectedThread#write(byte[])
     */
    public void write(byte[] out) {
        // se crea un objeto temporal
        ConnectedThread r;

        // sincroniza una copia the ConnectedThread
        Log.d(TAG, "write: Write Called.");

        if (bandera == 1){
            mConnectedThread.write(out);
        }



    }


}
