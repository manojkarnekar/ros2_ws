  
  const gpio = require('raspi-gpio');
  const InputEvent = require('input-event');

  const http = require('http');
  const scancode_URL = "http://192.168.10.201:8001/scancode?"

  const BarCodeTrigger = new gpio.DigitalOutput('GPIO27');
  const LockTrigger = new gpio.DigitalOutput('GPIO17');
  const LockFeed = new gpio.DigitalInput({
    pin: 'GPIO22',
    pullResistor: gpio.PULL_DOWN
  });

  var bScanner = null;  
  var num = "";
  var lockState = 1;
  var addMode = 1

  function onNumScan() {
   console.log(num);
   var url = scancode_URL + 'code=' + num + '&add=' + addMode;
   console.log(url)
   http.get(url, (res) => {
     //console.log(res)
   });
  }
 
  function onScan(a) {
   a = a.code;
   if(a>0 && a<=11) { num += (a-1)%10; }
   else { onNumScan(); num = ""; }
  }

  function openLocks() {
   lockState = 0
   LockTrigger.write(gpio.HIGH);
   setTimeout(() => { 
    LockTrigger.write(gpio.LOW) 
   },100)
  }

  function tryBindScan() {
   try {
    var input = new InputEvent('/dev/input/event0')
    bScanner = new InputEvent.Keyboard(input)
    bScanner.on('keyup' , onScan);
   }
   catch(e) {
    setTimeout(tryBindScan, 2000);
   }
  }

  function startScan() {
   BarCodeTrigger.write(gpio.HIGH);
   setTimeout(tryBindScan, 2000);
  }

  function stopScan() { 
   bScanner = null; 
   BarCodeTrigger.write(gpio.LOW);
  }

  function onLockClose() {
   stopScan();
   //console.log("Lock Closed")
  }

  function checkLockClose(val) {
   lockState = val
   if(val == 1) {
    LockFeed.removeAllListeners('change');
    onLockClose();
   }
  }

module.exports.init = () => {
 LockTrigger.write(gpio.LOW)
 BarCodeTrigger.write(gpio.LOW)
 //triggerLockScan();
}

module.exports.getLockState = () => {
 return lockState==1?0:1
}

module.exports.triggerLockScan = (add=1) => {
 addMode = add
 if(lockState == 1) {
  openLocks()
  startScan()
 }
 setTimeout(() => LockFeed.on('change', checkLockClose), 2000);
}
