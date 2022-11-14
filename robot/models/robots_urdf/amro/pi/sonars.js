const Gpio = require('pigpio').Gpio;

const MICROSECDONDS_PER_CM = 1e6/34321;
const sonars = [0,0,0,0,0];
const triggers = [
  new Gpio(21, {mode: Gpio.OUTPUT}),
  new Gpio(16, {mode: Gpio.OUTPUT}),
  new Gpio(23, {mode: Gpio.OUTPUT}),
  new Gpio(25, {mode: Gpio.OUTPUT}),
  new Gpio(26, {mode: Gpio.OUTPUT})
]

const echos = [
  new Gpio(20, {mode: Gpio.INPUT, alert: true}),
  new Gpio(13, {mode: Gpio.INPUT, alert: true}),
  new Gpio(18, {mode: Gpio.INPUT, alert: true}),
  new Gpio(24, {mode: Gpio.INPUT, alert: true}),
  new Gpio(19, {mode: Gpio.INPUT, alert: true})
]

const watchSonars = () => {
  echos.forEach( (echo, index) => {
    let startTick;
    echo.on('alert', (level, tick) => {
     if (level == 1) {
      startTick = tick;
     } else {
      const endTick = tick;
      const diff = (endTick >> 0) - (startTick >> 0);
      sonars[index] = (diff / 2 / MICROSECDONDS_PER_CM) >> 0;
      if(sonars[index]>200) sonars[index] = 0;
      //console.log("Sonar " + index + ": " + sonars[index]);
     }
    });
  });
};

module.exports.getArray = () => {
  return sonars;
}

module.exports.init = () => {
  triggers.forEach((trigger) => { 
    trigger.digitalWrite(0); 
  });
  watchSonars();
}

module.exports.update = () => {
  triggers.forEach((trigger) => { 
    trigger.trigger(10, 1); 
  });
  //console.log(JSON.stringify(sonars));
}
