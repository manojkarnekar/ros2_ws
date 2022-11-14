const roslib = require('roslib'),
    webSocket = require('ws'),
    os = require('os')

const WEBSERVER_URL = 'ws://jetbrain.ai:8000/robots',
    //const WEBSERVER_URL = 'ws://localhost:8000/robots',
    publishers = {},
    stats = {},
    externalStats = [],
    retryTimeout = 5,
    maxAttemptReport = 1;

var mac = "",
    module_name = "",
    ws = null,
    retryAttempt = 0

const getMAC = () => {
    var ifaces = os.networkInterfaces();
    var iface = ifaces['en0'] || ifaces['eth0'] || "";
    if (iface) {
        iface = iface.length ? iface[0] : iface;
        iface = iface["mac"] || "";
    }
    return iface
}

const sendMsg = () => {
    externalStats.forEach(addtionalStats =>
        Object.keys(addtionalStats).forEach(key => {
            stats[key] = addtionalStats[key]
        })
    )
    ws.send(JSON.stringify({
        stats: stats,
        mac: mac,
        module_name: module_name
    }))
}

const msgTimeout = () => {
    console.log("Message Timeout.")
    clearTimeout(ws.sendTimeout)
    clearTimeout(ws.pingTimeout)
    ws.terminate()
}

const msgLoop = () => {
    ws.sendTimeout = setTimeout(sendMsg, 5000)
    ws.pingTimeout = setTimeout(msgTimeout, 20000)
}

const onAPKinfoReq = (req, res) => {
    res.send({
        date_modified: Date.now(),
        apk_path: "http://192.168.10.50:2222/app-debug.apk",
        version: 2
    })
}

const onStatsReq = (req, res) => {

    externalStats.forEach(addtionalStats =>
        Object.keys(addtionalStats).forEach(key => {
            stats[key] = addtionalStats[key]
        })
    )

    res.send(JSON.stringify(stats));
}

const onExecCommands = (commands, params) => {
    commands = commands || []
    for (var c in commands) {
        console.log("Command Recieved : " + c.val)
        switch (c.val) {
            case 'move':
                var p = c.params
                moveBot(p.lspeed, p.aspeed)
        }
    }
}

const onWebSocketOpen = (d) => {
    ws.isOpen = true
    retryAttempt = 0
    console.log("Server Socket Open");
    sendMsg()
}

const onWebSocketClose = (d) => {
    ws.isOpen = false
    //console.log("Close", d)
    if (retryAttempt < maxAttemptReport)
        console.log("Webcon: Server Socket Closed." +
            ' Will silently retry every ' + retryTimeout +
            ' seconds...')
    retryAttempt++
    setTimeout(initSocket, retryTimeout * 1000)
}

const onWebSocketError = (d) => {
    ws.isOpen = false
    //console.log("Error", d)
    if (retryAttempt < maxAttemptReport)
        console.log("Webcon: Server Socket Error Occured.")
}

const onWebSocketMsg = (d) => {
    try {
        d = JSON.parse(d)
        if (d.isCommand) {
            onExecCommands(d.commands)
        }
    } catch (e) {

    }
    //console.log("Web Server Response: ", d);
    clearTimeout(ws.sendTimeout)
    clearTimeout(ws.pingTimeout)
    msgLoop()
}

const initSocket = () => {
    ws = new webSocket(WEBSERVER_URL, {
        perMessageDeflate: false
    });

    ws.on('open', onWebSocketOpen)
    ws.on('close', onWebSocketClose)
    ws.on('error', onWebSocketError)
    ws.on('message', onWebSocketMsg);
}

module.exports.appendStats = (addtionalStats) => {
    externalStats.push(addtionalStats)
}


module.exports.initPubSub = (ros) => {

}

module.exports.initRoutes = (app) => {
    app.get('/stats', onStatsReq)
    app.get('/apk-info', onAPKinfoReq)
}

module.exports.initDB = (dbCursor) => {

}

module.exports.init = (stats) => {
    mac = getMAC()
    module_name = process.env.MODULE_NAME || ""
    initSocket()
}
