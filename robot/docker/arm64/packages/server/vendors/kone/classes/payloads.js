var config;

module.exports.liftCallPayload = (buildingId, srcId, destId) => {

    const res = {
        type: 'lift-call-api-v2',
        buildingId: 'building:' + buildingId,
        callType: 'action',
        groupId: '1',
        payload: {
            request_id: 252390420,
            area: +srcId,
            time: '2022-03-10T07:17:33.298515Z',
            terminal: config.DEFAULT_TERMINAL,
            call: {
                action: 2,
                destination: +destId
            }
        }
    };

    if (config.FORCE_ALLOWED_LIFT.length)
        res.payload.call.allowed_lifts = config.FORCE_ALLOWED_LIFT;

    return res;
}

module.exports.liftCallPayload2 = (buildingId, srcId, destId) => {
    return {
        "type": "lift-call-api-v2",
        "buildingId": "building:" + buildingId,
        "callType": "action",
        "groupId": "1",
        "payload": {
            "request_id": 252390420,
            "area": 1001010,
            "time": "2022-03-10T07:17:33.298515Z",
            "terminal": config.DEFAULT_TERMINAL,
            "call": {
                "action": 5000,
                "destination": destId
            }
        }
    }
}

module.exports.holdDoorPayload = (buildingId, deck, area) => {
    return {
        "type": "lift-call-api-v2",
        "buildingId": "building:" + buildingId,
        "callType": "hold_open",
        "groupId": "1",
        "payload": {
            "time": "2020-10-10T07:17:33.298515Z",
            "lift_deck": deck, //1001010,
            "served_area": area, //5000,
            "hard_time": 10,
            "soft_time": 30
        }
    }
}

module.exports.siteMonitoringPayload = (buildingId, sessionId, subtopics = []) => {
    return {
        "type": "site-monitoring",
        "buildingId": "building:" + buildingId || config.DEFAULT_BUILDING_ID,
        "requestId": (Math.random() * 10000).toString(), //"01841d1c-f4ba-4f9c-a348-6f679bfae86e",
        "callType": "monitor",
        "groupId": "1",
        "payload": {
            "sub": config.CLIENT_ID,
            "duration": 300,
            "subtopics": subtopics
        }
    }
}

module.exports.siteConfigPayload = () => {
    const
        buildingId = config.DEFAULT_BUILDING_ID,
        groupId = config.DEFAULT_BUILDING_GROUP_ID;
    
    return {
        "type": "common-api",
        "requestId": "5e9e1d31-3d2c-42c7-907a-d8874117fb27",
        "buildingId": "building:" + buildingId,
        "callType": "config",
        "groupId": groupId
    }
};

module.exports.init = (configData) => {
    config = configData;
}
    
