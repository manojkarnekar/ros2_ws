const roslib = require('roslib'),
    publishers = {}

var faceData = [-1, -1]

function onFaceDataReq(req, res) {
    var data = {
        "face": {
            x: faceData[0],
            y: faceData[1]
        }
    }
    res.send(JSON.stringify(data))
}

function onFaceData(message) {
    faceData = message.data
}

module.exports.initPubSub = (ros) => {
    var faceDataListener = new roslib.Topic({
        ros: ros,
        name: '/facexy',
        messageType: 'std_msgs/Float32MultiArray'
    });
    faceDataListener.subscribe(onFaceData);
}

module.exports.initRoutes = (app) => {
    app.get('/face-xy', onFaceDataReq)
}

module.exports.initDB = (dbCursor) => {

}

module.exports.init = () => {}
