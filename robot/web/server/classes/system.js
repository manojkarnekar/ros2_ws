const roslib = require('roslib'),
    publishers = {},
    stats = {
        screens: []
    },
    spawnmode = {
        detached: true
    }

const {
    WebSocketServer
} = require('ws');

const stringify = (status, msg = "") => JSON.stringify({
    status: status
})

const {
    spawn
} = require('child_process');

var state = 0

const onStateData = (message) => {
    state = message.data
}

const onScreenReq = (d) => {

}

module.exports.update = () => {
    spawn('screen', ['-ls']).stdout.on('data', (data) => {
        stats.screens = data.toString().match(/sar.[a-z_]+/g) || [];
    });
}

module.exports.initScreenViewer = () => {
    const wss = new WebSocketServer({
        port: 8002
    });
    wss.on('connection', function connection(ws) {
        ws.on('message', function incoming(message) {
            console.log('received: %s', message);
        });

        ws.send('something');
    });
}

module.exports.getStatsObj = () => {
    return stats
}

module.exports.getState = () => {
    return state
}


const onShutdownReq = (req, res) => {
    console.log(req.query)
    const restart = +req.query.restart || 0
    var com = '/home/dev/sar/utilities/scripts/';
    switch (restart) {
        case 1:
            com += 'restart.sh';
            break;
        case 2:
            com += 'reset.sh';
            break;
        default:
            com += 'shutdown.sh';
            break;
    }
    
    var proc = spawn(com, [], spawnmode)
    proc.stdout.on('data', (data) => {});
    proc.on('error', (e) => { console.log(e); });
    res.send(stringify(1))
}

const onLocalizeReq = (req, res) => {
    console.log(req.query)
    var proc = spawn('rosservice', ['call', '/StartLocalization'], spawnmode)
    proc.stdout.on('data', (data) => {});
    proc.on('error', (e) => {});
    res.send(stringify(1))
}

module.exports.initPubSub = (ros) => {
    var stateListener = new roslib.Topic({
        ros: ros,
        name: '/amro_state',
        messageType: 'std_msgs/UInt8'
    });
    stateListener.subscribe(onStateData);
}

module.exports.initRoutes = (app) => {
    app.get('/screen', onScreenReq)
    app.get('/shutdown', onShutdownReq)
    app.get('/localize', onLocalizeReq)
}

module.exports.initDB = (dbCursor) => {

}

module.exports.initDB = (dbCursor) => {

}

module.exports.init = (dbCursor) => {}
