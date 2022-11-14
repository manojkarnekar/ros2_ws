const
    axios = require('axios'),
    WebSocket = require('ws'),
    payloads = require('./classes/payloads'),
    auth = require('./classes/auth');

var config = require('./config' +
    (process.env.DEBUG == "true" ? "-sim" : "") + '.json');

if (process.env.DEBUG == "true") console.log("===== DEBUG ON ======");

var accessToken = null,
    currentState = 0,
    apiTimeout = 20000,
    lastCall = 0,
    retrytimer = 0,
    retryCount = 0,
    siteConfig = {},
    liftAreaId;


const timedLog = (msg) => {
    console.log("Elevators " +
        Date().match(/\d+:\d+:\d+/)[0] + " : " + msg.toString())
}

const getSiteConfig = () => {
    var ws = new WebSocket(config.KONE_WEB_SOCKET_URL + accessToken, 'koneapi')

    ws.on('open', (data) => {
        timedLog("KONE Web Socket Connected")
        sendPayload(ws, payloads.siteConfigPayload());
    });

    ws.on('message', (data) => {
        try {
            data = JSON.parse(data);
        } catch (e) {
            timedLog("Error Parsing KONE API Response : " + JSON.stringify(data));
            return;
        }
        if ((data || {}).callType == "config") {
            timedLog("Recieved Building Config")
            siteConfig = data.data;
            //console.log(JSON.stringify(data));
            ws.close()
        }
    });
}

const sendPayload = (ws, payload) => {
    ws.send(JSON.stringify(payload));
};

const callElevator = (externalCB = null, buildingId, srcId, destId) => {
    buildingId = buildingId || config.DEFAULT_BUILDING_ID;
    srcId = srcId || "2000";
    destId = destId || "8000";

    currentState = 0;

    var destinations = (siteConfig.destinations || []).map(a => a.area_id);

    if (!accessToken) {
        timedLog("Access Token not Found.")
        return
    }

    timedLog("Robot Plan : Move from area " + srcId + " -> " + destId + " in building " + buildingId)
    timedLog("================================================================")
    timedLog("Making a new Web Socket Connection..")

    function timerFunc() {
        if (Date.now() - lastCall > apiTimeout) {
            ws.close();
            timedLog("KONE Web Socket Manually Disconnected.");
            if (retryCount < 3) {
                timedLog("Retrying (" + ++retryCount + ")...");
                try {
                    ws = new WebSocket(config.KONE_WEB_SOCKET_URL + accessToken, 'koneapi')
                    ws.on('open', onOpen);
                    ws.on('message', onMessage);
                    ws.on('error', onError);
                } catch (e) {
                    timedLog("Error: " + JSON.stringify(e));
                }
            } else {
                clearInterval(retrytimer);
            }
        }
    }

    function onError(e) {
        timedLog("KONE Web Socket Error : " + JSON.stringify(e))
    }

    function onOpen(data) {
        timedLog("KONE Web Socket Connected")
        if (destinations.indexOf(+srcId) > -1 && destinations.indexOf(+destId) > -1)
            sendPayload(ws, payloads.liftCallPayload(buildingId, srcId, destId))
        else
            timedLog("Source/Destination not found");
        currentState = 1
        externalCB(1);
        lastCall = Date.now();
    }

    function onMessage(data) {

        var resData, subtopic, callType, sessionId;

        timedLog(data);
        lastCall = Date.now();

        try {
            data = JSON.parse(data);
            resData = data.data || {};
            callType = data.callType;
            subtopic = data.subtopic;

        } catch (e) {
            timedLog("Error Parsing KONE API Response : " + e);
            return;
        }

        switch (callType) {
            case 'action':
                sessionId = resData.session_id;
                if (currentState <= 1 && sessionId) {
                    timedLog("Attempting Site monitoring...");

                    var call_states = ["registered", "assigned", "being_fixed",
                           "fixed", "being_served", "served", "served_soon", "cancelled"]

                    sendPayload(ws, payloads.siteMonitoringPayload(buildingId, sessionId,
                        call_states.map(cs => "call_state/" + sessionId + "/" + cs)))
                }
                break;
            case 'monitor':
                if ((subtopic || "").split("/")[2] == "fixed") {
                    liftAreaId = resData.allocated_lift_deck;

                    var group = siteConfig.groups
                        .filter(group => group["group_id"] == config.DEFAULT_BUILDING_GROUP_ID)[0],
                        lift = group.lifts
                        .filter(lift => lift.decks.filter(deck => deck["area_id"] == +liftAreaId).length > 0)[0];

                    timedLog("Deck Area:" + liftAreaId + " mathched to Lift:" + lift["lift_id"])

                    sendPayload(ws, payloads.siteMonitoringPayload(
                        buildingId, sessionId, ["lift_" + lift["lift_id"] + "/doors"]))
                }

                if ((subtopic || "").split("/")[1] == "doors") {
                    timedLog("Door State:" + resData.state + " Landing:" + resData.landing);
                    if (currentState < 3 && resData.landing == +srcId) {
                        externalCB(currentState = 2);
                        if (resData.state == 'OPENED') {
                            sendPayload(ws, payloads.holdDoorPayload(buildingId, +liftAreaId, +srcId))
                            externalCB(currentState = 3);
                        }
                    }
                    if (currentState < 5 && resData.landing == +destId) {
                        externalCB(currentState = 4);
                        if (resData.state == 'OPENED') {
                            externalCB(currentState = 5);
                            sendPayload(ws, payloads.holdDoorPayload(buildingId, +liftAreaId, +destId))
                            clearInterval(retrytimer);
                            ws.close();
                        }
                    }
                }

                break;
        }
    }

    clearInterval(retrytimer);
    retrytimer = setInterval(timerFunc, 5000);
    retryCount = 0;
    var ws = new WebSocket(config.KONE_WEB_SOCKET_URL + accessToken, 'koneapi')
    ws.on('open', onOpen);
    ws.on('message', onMessage);
}

const init = () => {

    auth.init(config);
    payloads.init(config);
    console.log(config);
    Promise.all([auth.fetchAccessToken()])
        .then(function (results) {
            accessToken = results[0];
            timedLog("KONE Web Socket Token Received")
            getSiteConfig()
            //timedLog(results);
        });
}

module.exports.callElevator = callElevator
module.exports.init = init
