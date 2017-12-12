package com.example.id341agk.myapplication;

import android.Manifest;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.os.SystemClock;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;
import android.widget.Chronometer;
import android.widget.TextView;
import android.widget.Toast;

import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.Locale;
import java.util.StringTokenizer;
import java.util.*;

public class MainActivity extends AppCompatActivity implements LocationListener, SensorEventListener{


    private LocationManager locationManager;
    private SensorManager sensorManager;

    private Button inizio,fine;
    private TextView t,lblInizio,lblFine,lblDistanza,stato,prova, accelerazione, giroscopio, magnetometro, percorso;

    private String report="LatitIniziale | LongitIniziale | LatitFinale | LongitFinale | DatiAccelerometro | " +
            "DatiMagnetomero | DatiGiroscopio | Distanza | Tempo   | Percorso\n";
    private Integer nProva=0;

    private double latitude, longitude;
    private double latInizio, lonInizio, latFine, lonFine, distanza, ax, ay, az, gx, gy, gz,magX, magY, magZ;
    public static DecimalFormat DECIMAL_FORMATTER;
    private String tempo;
    private List<Double>  DatiAccel = new ArrayList<Double>();
    private List<Double>  DatiMag = new ArrayList<Double>();
    private List<Double>  DatiGir = new ArrayList<Double>();

    //Definire il percorso da fare per la prova
    //private String[] way = {"1.0","2.0", "3.0", "4.0", "5.0", "Fine"};
    private String[] way = {"1m avanti", "2m a destra", "1m a sinistra"};

    //Cronometro per acqusire il tempo della prova
    private Chronometer cronometro;
    private Boolean actyvity = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_main);

        inizio =    (Button) findViewById(R.id.bInizio);
        fine =      (Button) findViewById(R.id.bFine);
        lblInizio =     (TextView) findViewById(R.id.lblInizio);
        lblFine =       (TextView) findViewById(R.id.lblFine);
        lblDistanza =   (TextView) findViewById(R.id.lblDistanza);
        stato =         (TextView) findViewById(R.id.lblStato);
        prova =         (TextView) findViewById(R.id.lblProva);
        t =             (TextView) findViewById(R.id.textView);
        cronometro = (Chronometer)findViewById(R.id.chronometer);
        accelerazione = (TextView) findViewById(R.id.textViewA);
        giroscopio = (TextView) findViewById(R.id.textViewG);
        magnetometro = (TextView) findViewById(R.id.textViewM);
        percorso = (TextView) findViewById(R.id.percorso);

        //Define decimal formatter
        DecimalFormatSymbols symbols = new DecimalFormatSymbols(Locale.US);
        symbols.setDecimalSeparator('.');
        DECIMAL_FORMATTER = new DecimalFormat("#.00", symbols);

        //Define manager to use the devise censor
        locationManager = (LocationManager) getSystemService(LOCATION_SERVICE);
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

        sensorManager.registerListener((SensorEventListener) this, sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener((SensorEventListener) this, sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE), SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener((SensorEventListener) this, sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_NORMAL);


        // START GPS
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                requestPermissions(new String[]{Manifest.permission.ACCESS_COARSE_LOCATION, Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.INTERNET}
                        , 10);
            }
            Toast.makeText(getApplicationContext(), "GPS NON ATTIVO", Toast.LENGTH_SHORT).show();
        }

        inizio.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                latInizio = latitude;
                lonInizio = longitude;

                lblInizio.setText("\nLon:" + lonInizio + "\nLat: " + latInizio);
                lblFine.setText("\nLon: \nLat: ");
                stato.setText("Vai...");
                lblDistanza.setText("\nDistanza: 0\n");
                percorso.setText(way[nProva]);

                if (!actyvity) {
                    cronometro.setBase(SystemClock.elapsedRealtime());
                    cronometro.start();
                }

            }
        });

        fine.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                lonFine = longitude;
                latFine = latitude;

                distanza = misuraGPS(latInizio, lonInizio, latFine, lonFine);
                String dist = String.format("%4.4f", distanza); //4 numeri decimali per la distanza
                cronometro.stop();
                long min = ((SystemClock.elapsedRealtime() - cronometro.getBase())/1000)/60;
                long sec =((SystemClock.elapsedRealtime() - cronometro.getBase())/1000)%60;
                long milisec = (SystemClock.elapsedRealtime() -cronometro.getBase())%1000;
                tempo = min +":"+ sec +":"+ milisec;
                lblFine.setText("\nLon:" + lonFine + "\nLat: " + latFine);
                stato.setText("Ricomincia...");
                lblDistanza.setText("Distanza: " + dist + " m");

                report = report + tempo+ " ; "+ latInizio + "; " + lonInizio + "; " + latFine + "; " + lonFine + "; "
                        +DatiAccel+"; "+DatiMag+"; "+DatiGir +"; "+ dist + "; " + way[nProva]+ "\r\n\r\n";
                //report = report + latInizio + "| " + lonInizio + "| " + latFine + "| " + lonFine + "| "
                        //+ dist + "| " + tempo +  way[nProva] +"\r\n";
                nProva = nProva + 1;
            }
        });

        locationManager.requestLocationUpdates("gps", 0, 0, this);

    }

    // Funzione per l'invio della email
    void sendEmail(String text){
        Intent i = new Intent(Intent.ACTION_SEND);
        i.setType("message/rfc822");
        i.putExtra(Intent.EXTRA_EMAIL  , new String[]{"scutelnicdumitru@gmail.com"});
        i.putExtra(Intent.EXTRA_SUBJECT, "Report");
        i.putExtra(Intent.EXTRA_TEXT   , text);

        startActivity(Intent.createChooser(i, "Invio mail..."));
    }

    // Funzione per il calcolo della distanza in metri
    private double misuraGPS(double lat1, double lon1,double lat2,double lon2){
        double R = 6378.137; // Raggio della terra in KM
        double dLat = lat2 * Math.PI / 180 - lat1 * Math.PI / 180;
        double dLon = lon2 * Math.PI / 180 - lon1 * Math.PI / 180;
        double a = Math.sin(dLat/2) * Math.sin(dLat/2) +
                Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
                        Math.sin(dLon/2) * Math.sin(dLon/2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
        double d = R * c;
        return d * 1000; // metri
    };

    @Override
    public void onLocationChanged(Location location) {
        t.setText("\nLon: " + location.getLongitude() + "\nLat: " + location.getLatitude());
        prova.setText("Prova Numero: " + nProva);

        latitude = location.getLatitude();
        longitude = location.getLongitude();

        if(nProva > way.length-1) {
            sendEmail(report);
            nProva = 0;
        }
    }

    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {
    }

    @Override
    public void onProviderEnabled(String provider) {
    }

    @Override
    public void onProviderDisabled(String s) {
        Toast.makeText(getApplicationContext(), "ATTIVA IL GPS", Toast.LENGTH_SHORT).show();
    }

    public void onAccuracyChanged(Sensor arg0, int arg1){
    }

    public void onSensorChanged(SensorEvent event){
        if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER){
            ax = event.values[0];
            ay = event.values[1];
            az = event.values[2];
            DatiAccel.add(ax);
            DatiAccel.add(ay);
            DatiAccel.add(az);

            String Ax = String.format("%.2f", ax);
            String Ay = String.format("%.2f", ay);
            String Az = String.format("%.2f", az);

            accelerazione.setText("x: " + Ax +" m/s\u00B2"+ "\ny:  " + Ay +" m/s\u00B2"+ "\nz: "+ Az+ " m/s\u00B2");
        }
        if(event.sensor.getType() == Sensor.TYPE_GYROSCOPE){
            gx = event.values[0];
            gy = event.values[1];
            gz = event.values[2];
            DatiGir.add(gx);
            DatiGir.add(gy);
            DatiGir.add(gz);

            String Gx = String.format("%.2f", gx);
            String Gy = String.format("%.2f", gy);
            String Gz = String.format("%.2f", gz);

            giroscopio.setText("x: " + Gx + " rad/s"+ "\ny:  " + Gy + " rad/s"+ "\nz: "+ Gz+ " rad/s");
        }
        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            // get values for each axes X,Y,Z
            magX = event.values[0];
            magY = event.values[1];
            magZ = event.values[2];
            DatiMag.add(magX);
            DatiMag.add(magY);
            DatiMag.add(magZ);

            String Mx = String.format("%.2f", magX);
            String My = String.format("%.2f", magY);
            String Mz = String.format("%.2f", magZ);

            double magnitude = Math.sqrt((magX * magX) + (magY * magY) + (magZ * magZ));
            magnetometro.setText("Mag "+DECIMAL_FORMATTER.format(magnitude) + " \u00B5Tesla"+"\nx: "
                    + Mx + " \u00B5Tesla"+ "\ny:  " + My + " \u00B5Tesla"+ "\nz: "+ Mz+ " \u00B5Tesla");
        }
    }

}

