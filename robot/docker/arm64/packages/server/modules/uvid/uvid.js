const LiveCam = require('livecam'),
	roslib = require('roslib'),
	ip = '0.0.0.0',
	publishers = {},
	conf = {
		'broadcast_addr': ip,
		'broadcast_port': 12000,
		'gst_addr': ip,
		'gst_port': 10000,
		'start': onStart,
		'webcam': {
			'grayscale': true,
			//'width': 800,
			//'height': 600,
			//'fake': true,
			'framerate': 0,
			'deviceIndex': 0
		},
		'exit': onExit
	}

var webcam_server;

function onStart() {
	console.log('WebCam server started!');
}

function onExit(c, e) {
	console.log("Starting LiveCam Again in 10 seconds..")
	setTimeout(init, 10000)
}

function onLightsData(req, res) {
	const isOn = req.query.on == 1 ? 1 : 0;
	const rosMsg = new roslib.Message({
		data: isOn
	});
	publishers.light_state.publish(rosMsg);
	res.send({
		status: 1
	});
}


module.exports.initPubSub = (ros) => {
	publishers.light_state = new roslib.Topic({
		ros: ros,
		name: '/light_state',
		messageType: 'std_msgs/UInt8'
	})
}

module.exports.initRoutes = (app) => {
	app.get('/lightstate', onLightsData)

}

module.exports.initDB = (dbCursor) => {

}

const init = () => {
	webcam_server = new LiveCam(conf);
	webcam_server.broadcast();
}

module.exports.init = init;
