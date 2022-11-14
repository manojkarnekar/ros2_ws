const api = require("../models/api"),
    rosdef = require("../models/rosdef"),
    roslib = require('roslib'),
    bmp = require("bmp-js"),
    fs = require('fs'),
    jimp = require('jimp'),
    tempPath = '/tmp',
    publishers = {},
    poseData = api.pose,
    poseStruct = rosdef.PoseWithCovarianceStamped

var db = null

const stringify = (status, msg = "") => JSON.stringify({
    status: status
})

const saveMap = (err, map) => {
    if (err) throw err;
    map //.resize(256, 256) 
        .flip(false, true)
        .quality(100)
        .greyscale()
        .write(tempPath + '/map.jpg');
}

const onMapData = (mapData) => {
    console.log("Received Map Data.")
    const raw = {
            width: mapData.info.width,
            height: mapData.info.height
        },
        mapArr = new Uint8Array(raw.width * raw.height * 4)

    mapData.data.forEach((p, i) => {
        if (p == 100) return;
        p = (p == -1) ? 0x88 : 0xFF;
        i = i * 4;
        mapArr[i] = mapArr[i + 1] = mapArr[i + 2] = mapArr[i + 3] = p;
    })
    raw.data = Buffer.from(mapArr)
    fs.writeFileSync(tempPath + "/map.bmp", bmp.encode(raw).data)
    jimp.read(tempPath + '/map.bmp', saveMap)
}

const onPoseData = (message) => {
    //console.log("Recieved Pose.")
    var pose = (message.pose || {}).pose || {}
    poseData.x = (pose.position || {}).x || 0
    poseData.y = (pose.position || {}).y || 0
    poseData.z = (pose.orientation || {}).z || 0
    poseData.w = (pose.orientation || {}).w || 0
}

const onSetGoalReq = (req, res) => {
    if (req.query.x >= 0 && req.query.y >= 0) {
        console.log(req.query.x, req.query.y);
        actionClients.currentGoal.send()
    }
    return res.send(stringify(1))
}

const onGetPoseReq = (req, res) => res.send(JSON.stringify(poseData))

const onSetPoseReq = (req, res) => {
    poseStruct.pose.pose.position.x = parseFloat(req.query.x) || 0
    poseStruct.pose.pose.position.y = parseFloat(req.query.y) || 0
    poseStruct.pose.pose.orientation.z = parseFloat(req.query.z) || 0
    poseStruct.pose.pose.orientation.w = parseFloat(req.query.w) || 0
    
    console.log('Publishing Initial Pose : ' + JSON.stringify(poseStruct))
    publishers.initPose.publish(new roslib.Message(poseStruct))
    return res.send(stringify(1))
}

const onMapReq = (req, res) => {
    console.log("Map Requested");
    if (fs.existsSync(tempPath + '/map.jpg')) {
        res.sendFile(tempPath + '/map.jpg');
    } else {
        console.log("No Map found.");
        return res.send(stringify(0))
    }
}

const onAddPath = (req, res) => {
    console.log(req.query)
    const
        waypoints = req.query.waypoints || [],
        d = (new Date()).toLocaleTimeString(),
        pathString = waypoints.join(",")

    if (typeof (waypoints) == "object" && waypoints.length >= 2) {
        db.get('SELECT Id FROM paths where PathString=?', pathString, (err, data) => {
            if (!err && !data) {
                const
                    source = waypoints[0],
                    target = waypoints[waypoints.length - 1],
                    q = 'INSERT INTO paths VALUES (?,?,?,?,?,?);',
                    args = [null, d, source, target, pathString, 1]

                try {
                    db.prepare(q, args).run().finalize(() => {
                        res.send(stringify(1))
                    })
                } catch (e) {
                    res.send(stringify(-1))
                }
            } else res.send(stringify(-1))
        })

    } else res.send(stringify(0))
}

const onAddWaypoint = (req, res) => {
    const
        d = (new Date()).toLocaleTimeString(),
        q = 'select Id from waypoints where PosX=? and PosY=?',
        q2 = 'INSERT INTO waypoints VALUES (?,?,?,?,?,?,?,?,?,?);',
        x = poseData.x, //req.query.x ,
        y = poseData.y, //req.query.y,
        z = poseData.z, //req.query.z || 0,
        w = poseData.w, //req.query.w || 0,
        floorId = req.query.floorid,
        name = req.query.name || "",
        type = req.query.type || 0,
        args = [null, d, type, name, floorId, x, y, z, w, 1]

    if (x != null && y != null && floorId != null && name != "") {
        db.get(q, [x, y], (err, data) => {
            if (!err && !data) {
                try {
                    db.prepare(q2, args).run().finalize(() => {
                        res.send(stringify(1))
                    });
                } catch (e) {
                    res.send(stringify(0))
                }
            } else res.send(stringify(!data ? 0 : -1))
        })
    } else res.send(stringify(0))
}

const onAddFloor = (req, res) => {
    const
        d = (new Date()).toLocaleTimeString(),
        q = 'select Id from floors where Name=?',
        q2 = 'INSERT INTO floors VALUES (?,?,?,?,?,?);',
		n = req.query.name || '',
        b = req.query.buildingid || 0,
        desc = req.query.description,
        args = [null, d,n,b,desc,1]

    if (n!='' && b>=0) {
        db.get(q, [n], (err, data) => {
			
				//console.log("-----" + err, data)
            if (!err && !data) {
                try {
                    db.prepare(q2, args).run().finalize(() => {
                        res.send(stringify(1))
                    });
                } catch (e) {
                    res.send(stringify(0))
                }
            } else res.send(stringify(!data ? 0 : -1))
        })
    } else res.send(stringify(0))
}


module.exports.init = (app) => {}

module.exports.initRoutes = (app) => {
    app.get('/map', onMapReq)
    app.get('/setpose', onSetPoseReq)
    app.get('/setgoal', onSetGoalReq)
    app.get('/getpose', onGetPoseReq)
    app.get('/addwaypoint', onAddWaypoint)
    app.get('/addpath', onAddPath)
    app.get('/addfloor', onAddFloor)
}

module.exports.initPubSub = (ros) => {
    rosdef.subscribers.mapListener.ros = ros;
    rosdef.subscribers.poseListener.ros = ros;
    rosdef.publishers.initPose.ros = ros;

    (new roslib.Topic(rosdef.subscribers.mapListener)).subscribe(onMapData);
    (new roslib.Topic(rosdef.subscribers.poseListener)).subscribe(onPoseData);
    publishers.initPose = new roslib.Topic(rosdef.publishers.initPose);
}

module.exports.initDB = (dbCursor) => {
    db = dbCursor
}
