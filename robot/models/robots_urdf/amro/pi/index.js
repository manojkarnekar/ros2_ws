const raspi = require('raspi');
const roslib = require('roslib');

const sonars = require('./sonars');
const barcodeLocks = require('./barcodeLocks')

const ROS_MASTER_URI = "http://192.168.10.201:11311/"

var ros = null;
var publishers = {}

const onStateData = (data) => {
   var state = data.data >> 0;
   if(state==30 || state==90) {
     barcodeLocks.triggerLockScan(state==30?1:0) 
   }
}

const initPubSub = () => {
    var stateListener = new roslib.Topic({
        ros: ros,
        name: '/amro_state',
        messageType: 'std_msgs/UInt8'
    });
    stateListener.subscribe(onStateData);

    //mapListener.unsubscribe();

    publishers.sonars = new roslib.Topic({
        ros: ros,
        name: '/module_sensors',
        messageType: 'std_msgs/UInt8MultiArray'
    });

    publishers.lockState = new roslib.Topic({
        ros: ros,
        name: '/lockstate',
        messageType: 'std_msgs/UInt8'
    });
}

const initROS = () => {
    ros = new roslib.Ros({
        url: ROS_MASTER_URI.replace("http", "ws").replace("11311", "9090"),
        encoding: 'utf8'
    });
    ros.on('connection', () => {
        console.log('Connected to ROS server');   
    });
    ros.on('error', (error) => {
        console.log("Error Connecting to ROS.." + error)
    });
    ros.on('close', () => {
        console.log('Connection to ROS server closed. Retrying in 5 Sec..');
        setTimeout(initROS, 5000);
    });

    initPubSub();
}

const init = () => {
  sonars.init()
  barcodeLocks.init()
  initROS()
}

const update = () => {
  sonars.update()
  //console.log(sonars.getArray()) 
  var sonarArray = new roslib.Message({
    data:sonars.getArray()
  });
  publishers.sonars.publish(sonarArray)

  var lockStateVal = new roslib.Message({
    data:barcodeLocks.getLockState()
  });
  publishers.lockState.publish(lockStateVal)
}


raspi.init(() => {
  init()
  setInterval(update, 1000)
});


