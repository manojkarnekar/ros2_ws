var roslib = require('roslib'),
    publishers = {},
    api = null,
    elevatorState = 0;

const onElevatorStateUpdate = (estate) => {
    elevatorState = estate
    var rm = new roslib.Message({
        data: elevatorState
    })
    if (publishers.liftState)
        publishers.liftState.publish(rm);

    console.log("Current State : " + estate);
}

const getStatus = function (req, res) {
    console.log();
}

const onElevatorCall = function (req, res) {
    var q = req.query;
    console.log("Manual Elevator Call : " + JSON.stringify(req.query));
    api.callElevator(onElevatorStateUpdate, q.bid, q.src, q.dst);
    res.send({
        status: 1,
        msg: "success"
    })
}

module.exports.setVendorAPI = (vendorAPI) => {
    vendorAPI.init()
    api = vendorAPI;
}

module.exports.update = (state) => {
    if (state == 180 && (elevatorState == 0 || elevatorState == 5)) {
        elevatorState = 1
        api.callElevator(onElevatorStateUpdate);
    }
}

module.exports.initPubSub = (ros) => {
    publishers.liftState = new roslib.Topic({
        ros: ros,
        name: '/lift_state',
        messageType: 'std_msgs/Int8'
    })
}

module.exports.initRoutes = (app) => {
    app.get('/elevator/call', onElevatorCall)
    app.get('/elevator/status', getStatus)
}

module.exports.initDB = (dbCursor) => {

}

module.exports.initDB = (dbCursor) => {

}

module.exports.init = (dbCursor) => {}
