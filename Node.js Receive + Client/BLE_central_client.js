
// NOBLE Bluetooth 
var noble = require('../..');             //noble variable
var testServiceUuid = 'fff0';             //service UUID
var xCharacteristicUuid = 'fff1';      //characteristic UUID
var yCharacteristicUuid = 'fff2';      //characteristic UUID
var testService = null;                   //service variable
var xCharacteristic = null;            //charactertistic variable
var yCharacteristic = null;            //charactertistic variable

// CLIENT to send data to MATLAB
client = require('./BLE_client');
host='localhost';
port=7070;
c = new client(host, port);
c.receive();



noble.on('stateChange', function(state) {
  if (state === 'poweredOn') {
    //
    // Once the BLE radio has been powered on, it is possible
    // to begin scanning for services. Pass an empty array to
    // scan for all services (uses more time and power).
    //
    console.log('scanning...');
    noble.startScanning([testServiceUuid], false);
  }
  else {
    noble.stopScanning();
  }
})


noble.on('discover', function(peripheral) {
  // we found a peripheral, stop scanning
  noble.stopScanning();

  //
  // The advertisment data contains a name, power level (if available),
  // certain advertised service uuids, as well as manufacturer data,
  // which could be formatted as an iBeacon.
  //
  console.log('found peripheral:', peripheral.advertisement);
  //
  // Once the peripheral has been discovered, then connect to it.
  //
  peripheral.connect(function(err) {
    //
    // Once the peripheral has been connected, then discover the
    // services and characteristics of interest.
    //
    peripheral.discoverServices([testServiceUuid], function(err, services) {
      services.forEach(function(service) {
        //
        // This must be the service we were looking for.
        //
        console.log('found service:', service.uuid);

        //
        // So, discover its characteristics.
        //
        service.discoverCharacteristics([], function(err, characteristics) {

          characteristics.forEach(function(characteristic) {
            //
            // Loop through each characteristic and match them to the
            // UUIDs that we know about.
            //
            console.log('found characteristic:', characteristic.uuid);

            if (xCharacteristicUuid == characteristic.uuid) {
              xCharacteristic = characteristic;
            }
            if (yCharacteristicUuid == characteristic.uuid) {
              yCharacteristic = characteristic;
            }
            
          })

          if (xCharacteristic) { 

            console.log('characteristic: ' + xCharacteristic.name);
            
            xCharacteristic.read(function(error, data) {

              if (data) {

                //var string = data.toString('ascii');

                //console.log('Data: ' + string);

                xCharacteristic.notify(true);

                xCharacteristic.on('notify', function(isNotifying) {
                    console.log('new notification!');
                });

                xCharacteristic.on('data', onCharacteristicData);

                
              }
                        
            });
          }
          else {
            console.log('missing X characteristics');
          }

          if (yCharacteristic) { 

            console.log('characteristic: ' + yCharacteristic.name);
            
            yCharacteristic.read(function(error, data) {

              if (data) {

                //var string = data.toString('ascii');

                //console.log('Data: ' + string);

                yCharacteristic.notify(true);

                yCharacteristic.on('notify', function(isNotifying) {
                    console.log('new notification!');
                });

                yCharacteristic.on('data', onCharacteristicData);

                
              }
                        
            });
          }
          else {
            console.log('missing Y characteristics');
          }

        })
      })
    })


  })


})


var value = 0, xvalue = 0, yvalue = 0, bb = 0, yy = 0, location = [0, 0];
function onCharacteristicData(data, isNotification){

    value = 0;

    for (i = 0; i < data.length; i++) { 
      bb = 256**(i);
      value += (bb)*data[i];
      //console.log('BLE Data: i=' + i + " " + data[i] + " value: " + bb + "*" + data[i] + "=" + value);
    }

    if(yy == 0) {
      yy = 1;
      xvalue = value/1000;
    }
    else if(yy = 1){
      yvalue = value/1000;
      console.log('BLE Data: X:' + xvalue + " " + " Y:" + yvalue + '\n');
      yy = 0;

      location[0] = xvalue; location[1] = yvalue; 
      c.send(location);
    }
    
}
