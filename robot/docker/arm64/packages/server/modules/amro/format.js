module.exports.floorRoomData = (data) => {
    floors = {}
    data.forEach((row) => {
        var floorId = row['floorId']
        var floorName = row['floorName']

        floors[floorId] = floors[floorId] || {}
        floors[floorId].floorId = floorId
        floors[floorId].floorName = floorName
        floors[floorId].floorRooms = floors[floorId].floorRooms || []

        floors[floorId].floorRooms.push({
            "roomId": row['waypointId'],
            "roomName": row['waypointName']
        })
    })

    return {
        "floors": Object.values(floors)
    }
}

module.exports.stockData = (data) => {
    return {
        amroStock: data
    }
}

module.exports.userData = (data) => {
    return {
        userId: data['Id'],
        userName: data['Name']
    }
}

module.exports.tripData = (data) => {
    return {
        tripId: data['Id'],
        tripStatus: data['StatusId']
    }
}

module.exports.statusData = (data) => {
    return {
        status: data
    }
}

/*module.exports.deviceStatData = (statdata) => {
    var data = {}
    statdata.forEach((i) => {
        data[statdata[i][1]] = statdata[i][2]
    })
    return data
}*/
