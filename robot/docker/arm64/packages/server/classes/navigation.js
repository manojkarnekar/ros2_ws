var db = null

const get_floors = (cb) => {
	db.all('SELECT ' +
		'floors.Id as floorId,' +
		'floors.Name as floorName ' +
		'FROM floors', [], cb)
}

const get_waypoint_types = (cb) => {
	db.all('SELECT ' +
		'waypoint_types.Id as Id,' +
		'waypoint_types.Name as Name ' +
		'FROM waypoint_types', [], cb)
}

const get_waypoint_floors = (cb, waypointIds) => {
	var q = 'SELECT DISTINCT ' +
		'floors.Id as floorId,' +
		'floors.Name as floorName ' +
		'FROM floors ' +
		'JOIN waypoints on floors.Id=waypoints.FloorId ' +
		'WHERE waypoints.Id in (' + waypointIds + ')'
	
	db.all(q, [waypoints], cb);
}

const get_waypoints = (cb, floorId, type) => {
	var q = 'SELECT ' +
		'waypoints.Id as Id,' +
		'waypoints.Name as Name ' +
		'FROM waypoints ' +
		'WHERE FloorId=?',
		p = [floorId]

	if (type >= 0) {
		q += 'and Type=?'
		p.push(type)
	}
	db.all(q, p, cb)
}


const get_waypoint_id_names = (cb, waypointIds) => {
	var q = 'SELECT ' +
		'waypoints.Id as Id,' +
		'waypoints.Name as Name, ' +
		'waypoints.FloorId as FloorId, ' +
		'floors.Name as FloorName ' +
		'FROM waypoints ' +
		'LEFT JOIN floors on Floors.Id=FloorId ' +
		'WHERE waypoints.Id in (' + waypointIds + ')'
	db.all(q, [], cb)
}

const get_paths = (cb) => {
	var q = 'SELECT ' +
		'Id as pathId,' +
		'PathString as pathString ' +
		'FROM paths '
	db.all(q, [], cb)
}


const natural = (val) => {
	return val>=0?val:-1;
}

const floors = (req, res) => {
	var srcId = natural(req.query.sourcewaypointid)
	if (srcId == -1) {
		get_floors((err, data) => {
			res.send(JSON.stringify({
				floors: data
			}))
		})
	} else {
		get_paths((err, pathdata) => {
			waypointList = []
			pathdata = pathdata.forEach(pd=> {
				var wps = pd.pathString.split(",").slice(0,-1);
				if(wps.indexOf(srcId) >= 0) 
					waypointList = waypointList.concat(wps);
			});
			waypointList =  Array.from(new Set(waypointList));
			get_waypoint_floors((err, data) => {
				res.send(JSON.stringify({
					floors: data
				}))
			},waypointList)
		})
	}
}

const waypoints = (req, res) => {
	var srcId = natural(req.query.sourcewaypointid)
	var floorId = natural(req.query.floorid)
	var type = natural(req.query.type)
	if (srcId == -1) {
		get_waypoints((err, data) => {
			res.send(JSON.stringify({
				waypoints: data
			}))
		}, floorId, type)
	}
	else {
		get_paths((err, pathdata) => {
			waypointList = []
			pathdata = pathdata.forEach(pd=> {
				var wps = pd.pathString.split(","),
					wpi = wps.indexOf(srcId);
				if(wpi>-1 && wpi<(wps.length-1)) 
					waypointList = waypointList.concat(wps.filter((wp,i)=> i>wpi));
			});
			waypointList = new Set(waypointList);
			
			get_waypoints((err, data) => {
				data = data.filter(a=> waypointList.has(a.Id.toString()));
				res.send(JSON.stringify({
					waypoints: data
				}))
			}, floorId, type)
		})
	}
}


const pathwaypoints = (req, res) => {
	get_paths((err, paths) => {
		var list = paths.map(k => k.pathString).join(",")
		get_waypoint_id_names((err, waypoints) => {
			var wp = {},
				pathData = []

			waypoints.forEach(w => wp[w.Id] = w)
			paths.forEach(p =>
				pathData.push({
					pathId: p.pathId,
					waypoints: p.pathString.split(",").map(w => wp[w])
				})
			)
			res.send(JSON.stringify({
				paths: pathData
			}))
		}, list)
	})
}

const paths = (req, res) => {
	get_paths((err, data) => {
		res.send(JSON.stringify({
			paths: data
		}))
	})
}

const waypointtypes = (req, res) => {
	get_waypoint_types((err, data) => {
		res.send(JSON.stringify({
			waypointtypes: data
		}))
	})
}

module.exports.init = (app) => {}

module.exports.initRoutes = (app) => {
	app.get("/waypoints", waypoints)
	app.get("/paths", paths)
	app.get("/pathwaypoints", pathwaypoints)
	app.get("/floors", floors)
    app.get('/waypointtypes', waypointtypes)
}

module.exports.initPubSub = (ros) => {}

module.exports.initDB = (dbCursor) => {
	db = dbCursor
}
