const roslib = require('roslib'),
    publishers = {}

var botState = 0

const minX = -130,
    minY = 0,
    maxX = 130,
    maxY = 255,
    scaleX = -1,
    scaleY = -1;

const onBotState = (req, res) => {
    botState = +req.query.state ? 1 : 0;
    console.log("Bot State : " + botState);
    publishers.botstate.publish(new roslib.Message({
        data: botState
    }));
    res.send(JSON.stringify({
        status: 1
    }));
}

const onWebJoystickData = (req, res) => {
    console.log("Requested Move : " + JSON.stringify(req.query));

    var dx = req.query.dx,
        dy = req.query.dy;

    dx *= scaleX;
    dy *= scaleY;

    dx = Math.min(Math.max(dx, minX), maxX);
    dy = Math.min(Math.max(dy, minY), maxY);
    
    dx /= 100;

    publishers.steering.publish(new roslib.Message({
        data: dx
    }));
    publishers.throttle.publish(new roslib.Message({
        data: dy
    }));
    
    console.log("Publishing X,Y:" + dx + "," + dy);

    res.send(JSON.stringify({
        status: 1
    }));
}

module.exports.init = (app) => {}

module.exports.initRoutes = (app) => {
    app.get('/move-afs', onWebJoystickData)
    app.get('/botstate', onBotState)
}

module.exports.initPubSub = (ros) => {
    publishers.throttle = new roslib.Topic({
        ros: ros,
        name: '/throttle_state',
        messageType: 'std_msgs/UInt16'
    })
    publishers.botstate = new roslib.Topic({
        ros: ros,
        name: '/bot_state',
        messageType: 'std_msgs/UInt8'
    })
    publishers.steering = new roslib.Topic({
        ros: ros,
        name: '/heading',
        messageType: 'std_msgs/Float32'
    })
}

module.exports.initDB = (dbCursor) => {

}
