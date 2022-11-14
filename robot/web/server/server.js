const path = require('path'),
    express = require('express'),
    app = express(),
    roslib = require('roslib'),
    sqlite3 = require('sqlite3').verbose()

const MASTER_URI = process.env.ROS_MASTER_URI || "http://localhost:11311"

const
    system = require('./classes/system'),
    elevators = require('./classes/elevators'),
    sarviz = require('./classes/sarviz'),
    teleop = require('./classes/teleop'),
    webcon = require('./classes/webcon'),
    battery = require('./classes/battery'),
    nav = require('./classes/navigation'),
    amro = require('./modules/amro/amro'),
    arya = require('./modules/arya/arya'),
    uvid = require('./modules/uvid/uvid'),
    afs = require('./modules/afs/afs'),
    kone = require('./vendors/kone/kone');

var ws,
    db,
    publishers = {},
    initClasses

const initDB = () => {
    var dbPath = process.env.DB_PATH
    if (!dbPath) console.log("DB_PATH not set.")
    else db = new sqlite3.Database(dbPath, (err) => {
        if (err) {
            console.log("Unable to open database file : " + dbPath)
            db = null
        } else {
            console.log("DB Connected.")
            initClasses.forEach(c => {
                c.initDB(db)
            })
        }
    })
}

const initPubSub = (ros) => {
    initClasses.forEach(c => {
        c.initPubSub(ros)
    })
}

const initROS = (retryAttempt) => {
    var maxAttemptReport = 1;
    var retryTimeout = 5;
    var url = MASTER_URI.replace("http", "ws").replace("11311", "9090");
    var ros = new roslib.Ros({
        url: url,
        encoding: 'utf8'
    })
    ros.on('connection', () => {
        retryAttempt = 0
        console.log('Connected to ROS server')
        initPubSub(ros)
    })
    ros.on('error', (error) => {
        if (retryAttempt < maxAttemptReport && error)
            console.log("Error Connecting to ROS.." + error.message)
    })
    ros.on('close', () => {
        if (retryAttempt < maxAttemptReport)
            console.log('Connection to ROS server closed.' +
                ' Will silently retry every ' + retryTimeout +
                ' seconds...')
        setTimeout(() => {
            initROS(++retryAttempt)
        }, retryTimeout * 1000)
    })
}

const initRoutes = () => {
    initClasses.forEach(c => {
        c.initRoutes(app)
    })
}

const update = () => {
    system.update()
    elevators.update(system.getState())
}

const init = () => {

    initClasses = [sarviz, elevators, teleop, battery, system, nav]

    if (process.env.SAR_WEB_CONSOLE != 0)
        initClasses.push(webcon);

    switch (process.env.MODULE_NAME) {
        case 'AMRO':
            initClasses.push(amro);
            break;
        case 'ARYA':
            initClasses.push(arya);
            break;
        case 'UVID':
            initClasses.push(uvid);
            break;
        case 'AFS':
            initClasses.push(afs);
            break;
        default:
            console.log("No Modules added..")
    }

    initClasses.forEach(c => {
        c.init()
    })

    elevators.setVendorAPI(kone);

    webcon.appendStats(battery.getStatsObj())
    webcon.appendStats(system.getStatsObj())

    initROS(1)
    initDB()
    initRoutes()


    app.use(express.static(path.join(__dirname, 'web')))
    app.listen(8001)
    setInterval(update, 2000)
}

init()
