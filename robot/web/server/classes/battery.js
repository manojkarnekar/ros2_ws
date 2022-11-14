const
    roslib = require('roslib'),
    rosdef = require("../models/rosdef"),
    battery = require("../models/battery"),
    {
        SerialPort
    } = require('serialport'),
    {
        ReadlineParser
    } = require('@serialport/parser-readline'),
    publishers = {},
    stats = {
        battery: {
            P: -1
        }
    }

var port, parser;

const onData = d => {
    const list = battery.parseList,
        k = d.split("\t");
    if (!!list[k[0]]) {
        stats.battery[k[0]] = list[k[0]].isNumber ? +k[1] : k[1];
        var rm = new roslib.Message({
            data: +k[1]
        });
        if (publishers.battery && k[0] == 'P') publishers.battery.publish(rm);
    }
}

const onError = e => {
    console.log("Error connecting to Battery Monitor..");
}

module.exports.getStatsObj = () => {
    return stats
}

module.exports.init = () => {
    port = new SerialPort(battery.connectionString);
    port.on('error', onError);
    parser = port.pipe(new ReadlineParser(battery.delimiter));
    parser.on('data', onData);
}

module.exports.initPubSub = (ros) => {
    rosdef.publishers.battery.ros = ros
    publishers.battery = new roslib.Topic(rosdef.publishers.battery)
}

module.exports.initRoutes = (app) => {

}

module.exports.initDB = (dbCursor) => {

}
