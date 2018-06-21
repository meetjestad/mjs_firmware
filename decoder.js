//From https://github.com/thesolarnomad/lora-serialization/blob/master/src/decoder.js
var bytesToInt = function(bytes) {
  var i = 0;
  for (var x = 0; x < bytes.length; x++) {
    i |= +(bytes[x] << (x * 8));
  }
  return i;
};

function Decoder(bytes, port) {
    var decoded = {};

    switch(port) {
      case 21: // Rangetester data
        // Lat-lon is stored as 32768ths, so divide to get degrees.
        decoded.latitude = bytesToInt(bytes.slice(0, 3).reverse()) / 32768;
        decoded.longitude = bytesToInt(bytes.slice(3, 6).reverse()) / 32768;
        decoded.altitude = bytesToInt(bytes.slice(6, 10).reverse()) / 100; // cm
        decoded.hdop = bytesToInt(bytes.slice(10, 12).reverse()) / 1000; // thousands of a DOP
        break;
    }
    return decoded;
}
