const db = require('./db'),
    format = require('./format'),
    publishers = {},
    roslib = require('roslib'),
    rosdef = require("../../models/rosdef")

const send = (res, msg) => {
    res.send(JSON.stringify(msg))
}

const amroStock = (req, res) => {
    var tripId = req.query.tripid || -1
    db.get_stock_from_tripId((data) => {
        send(res, format.stockData(data))
    }, tripId)
}

const floorRooms = (req, res) => {
    db.get_floor_and_rooms((data) => {
        send(res, format.floorRoomData(data))
    })
}

const dispatch = (req, res) => {
    var tripId = req.query.tripid || -1
    var pin = req.query.pin || -1
    db.get_user_from_pin((userdata) => {
        if (userdata && userdata.Id) {
            db.start_amro_trip((data) => {
                if (data != 1) userdata = {}
                send(res, format.userData(userdata))
            }, userdata.Id, tripId)
            console.log(userdata)
        }
    }, pin)
}

const delivery = (req, res) => {
    var tripId = req.query.tripid || -1
    var pin = req.query.pin || -1
    db.get_user_from_pin((userdata) => {
        if (userdata && userdata.Id) {
            db.end_amro_trip((data) => {
                if (data != 1) userdata = {}
                send(res, format.userData(userdata))
            }, userdata.Id, tripId)
            console.log(userdata)
        }
    }, pin)
}

const cancelDelivery = (req, res) => {
    var tripId = req.query.tripid || -1
    var pin = req.query.pin || -1
    db.get_user_from_pin((userdata) => {
        if (userdata && userdata.Id) {
            publishers.move_base_cancel.publish(new roslib.Message(rosdef.GoalID))
            db.canceldelivery((data) => {
                if (data != 1) userdata = {}
                send(res, format.userData(userdata))
            }, userdata.Id, tripId)
            console.log(userdata)
        }
    }, pin)
}

const setGoal = (req, res) => {
    var tripId = req.query.tripid || -1
    var goalId = req.query.goalid || -1
    var sourcewaypointid = req.query.sourcewaypointid || -1
    console.log(sourcewaypointid, goalId)
    db.set_trip_goal((data) => {
        send(res, format.statusData(data))
    }, tripId, sourcewaypointid, goalId)
}

const amroState = (req, res) => {
    db.get_trip_status((data) => {
        send(res, format.tripData(data))
    })
}

const newTrip = (req, res) => {
    db.create_new_trip((data) => {
        send(res, format.statusData(data))
    })
}

const setBreak = (req, res) => {
    var tripId = req.query.tripid || -1
    var stop = req.query.stop != '0'
    db.setbreaks((data) => {
        send(res, format.statusData(data))
    }, tripId, stop)
}

const onBarcodeScan = (req, res) => {
    var bid = req.query.code
    var addMode = req.query.add != '0'
    db.get_current_tripId((trip) => {
        tripId = trip.Id
        if (bid && tripId) {
            db.get_stock_from_barcode((stock) => {
                if (stock) db.update_amro_stock((data) => {
                    res.send(JSON.stringify({
                        status: 1
                    }));
                }, addMode, stock.Id, tripId)
            }, bid)
        }
    })
}

module.exports.initRoutes = (app) => {
    app.get("/state", amroState)
    app.get("/floors-rooms", floorRooms)
    app.get("/dispatch", dispatch)
    app.get("/delivery", delivery)
    app.get("/cancel-delivery", cancelDelivery)
    app.get("/set-goal", setGoal)
    app.get("/newtrip", newTrip)
    app.get("/set-break", setBreak)
    app.get("/scancode", onBarcodeScan)
    app.get("/amro-stock", amroStock)
    app.get("/amro-state", amroState)
    //app.get("/device-stats",testFunc) 
}

module.exports.initDB = (dbCursor) => {
    db.setDBCursor(dbCursor)
}


module.exports.init = () => {}
module.exports.initPubSub = (ros) => {
    rosdef.publishers.move_base_cancel.ros = ros;
    publishers.move_base_cancel = new roslib.Topic(rosdef.publishers.move_base_cancel);
}
