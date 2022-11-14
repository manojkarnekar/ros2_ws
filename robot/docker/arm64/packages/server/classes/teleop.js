const roslib = require('roslib'),
    {
        SerialPort
    } = require('serialport'),
    {
        ReadlineParser
    } = require('@serialport/parser-readline'),
    publishers = {}

var moveTimeout = 0

const moveBot = (lspeed = 0, aspeed = 0) => {
    clearTimeout(moveTimeout);

    var twist = new roslib.Message({
        linear: {
            x: 0.0,
            y: 0.0,
            z: 0.0
        },
        angular: {
            x: 0.0,
            y: 0.0,
            z: 0.0
        }
    });

    twist.linear.x = lspeed;
    twist.angular.z = aspeed;

    if (lspeed != 0 || aspeed != 0)
        moveTimeout = setTimeout(moveBot, 5000);

    if (publishers.cmdVel) {
        publishers.cmdVel.publish(twist);
        console.log("Publishing.." + JSON.stringify(twist))
    }

    return 1;
}

const onWebJoystickData = (req, res) => {
    console.log("Requested Move : " + JSON.stringify(req.query));
    var dx = req.query.dx,
        dy = req.query.dy;

    dx *= -0.5;
    dy *= -0.5;

    if (dx > 100) dx = 100;
    if (dy > 100) dy = 100;
    if (dx < -100) dx = -100;
    if (dy < -100) dy = -100;

    var aspeed = 0,
        lspeed = 0;

    if (Math.abs(dx) > 0) aspeed = 0.8 * dx / 100;
    if (Math.abs(dy) > 0) lspeed = 0.3 * dy / 100;

    res.send(JSON.stringify({
        status: +moveBot(lspeed, aspeed)
    }));
}

const onHWJoystickData = (d) => {
    var v = d.split(",")
    var x = (Math.round(+v[0] / 1024 * 10) - 5)
    var y = (Math.round(+v[1] / 1024 * 10) - 5)
    var aspeed = 0,
        lspeed = 0;
    if (Math.abs(x) > 0) aspeed = 0.8 * x / 10;
    if (Math.abs(y) > 0) lspeed = 0.47 * y / 10;
    moveBot(lspeed, aspeed)
}

const onHWJoystickError = (e) => {
    console.log("No physical joystick found..");
}

const initHWJoystick = () => {
    const port = new SerialPort({
            path: '/dev/joystick',
            baudRate: 9600
        }),
        parser = port.pipe(new ReadlineParser({
            delimiter: '\r\n'
        }));
    parser.on('data', onHWJoystickData);
    port.on('error', onHWJoystickError);
}

module.exports.init = (app) => {
    if (process.env.HW_JOYSTICK != 0) initHWJoystick();
}

module.exports.initRoutes = (app) => {
    app.get('/move', onWebJoystickData)
}

module.exports.initPubSub = (ros) => {
    publishers.cmdVel = new roslib.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });
}

module.exports.initDB = (dbCursor) => {

}
