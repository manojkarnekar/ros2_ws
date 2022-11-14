var dbCursor = null

module.exports.setDBCursor = (cursor) => {
    dbCursor = cursor
}

module.exports.get_floor_and_rooms = (cb) => {
    dbCursor.all('SELECT ' +
        'waypoints.Id as waypointId, ' +
        'waypoints.Name as waypointName,' +
        'floors.Id as floorId,' +
        'floors.Name as floorName ' +
        'FROM waypoints ' +
        'LEFT JOIN floors ' +
        'ON floors.Id=waypoints.FloorId', [], (err, data) => {
            cb(data)
        })
}

module.exports.get_current_tripId = (cb) => {
    dbCursor.get('select Id ' +
        'FROM amro_trips ' +
        'ORDER BY Id ' +
        'DESC LIMIT 1;', [], (err, data) => {
            cb(data)
        })
}

module.exports.get_trip_status = (cb) => {
    dbCursor.get('SELECT Id,StatusId ' +
        'FROM amro_trips ORDER BY Id DESC LIMIT 1;', [], (err, data) => {
            cb(data)
        })
}

module.exports.get_stock_from_tripId = (cb, tripId) => {
    dbCursor.all('SELECT ' +
        'stock.Id as stockId,' +
        'stock.Name as stockName,' +
        'amro_stock.Quantity as stockQuantity,' +
        'TripId as tripId ' +
        'FROM amro_stock ' +
        'LEFT JOIN stock ' +
        'ON stock.Id=amro_stock.StockId WHERE TripId=?', tripId, (err, data) => {
            cb(data)
        })
}

module.exports.get_stock_from_barcode = (cb, barcode) => {
    dbCursor.get('SELECT * FROM stock WHERE barcode=?', barcode, (err, data) => {
        cb(data)
    })
}

module.exports.get_user_from_pin = (cb, pin) => {
    dbCursor.get('select Id,Name from users where Passcode=?', pin, (err, data) => {
        cb(data)
    })
}

module.exports.set_trip_goal = (cb, tripId, sourceId, targetId) => {
    dbCursor.all('SELECT * FROM PATHS', (err, data) => {
		paths = (data || []).filter(d=> {
			var wps = d.PathString.split(",");
			return wps.indexOf(sourceId)==0 && wps.indexOf(targetId)==(wps.length-1);
		})
		
		if(!paths.length) cb(-1)
		
		else {
			var q = 'UPDATE amro_trips SET PathId=? where Id=?;'
			var stmt = dbCursor.prepare(q, [paths[0].Id, tripId])
			stmt.run();
			stmt.finalize();
			cb(1)
		}
    })
	
	
}

module.exports.create_new_trip = (cb) => {
    var q = 'INSERT INTO amro_trips VALUES (?,?,?,?,?,?);'
    var args = [null, (new Date()).toLocaleTimeString(), '', '', '', 20]
    var stmt = dbCursor.prepare(q, args)
    stmt.run();
    stmt.finalize();
    cb(1)
}

module.exports.start_amro_trip = (cb, userId, tripId) => {
    var q = 'UPDATE amro_trips SET InitiatedBy=?,StatusId=? where Id=?;'
    var stmt = dbCursor.prepare(q, [userId, 30, tripId])
    stmt.run();
    stmt.finalize();
    cb(1)
}

module.exports.end_amro_trip = (cb, userId, tripId) => {
    var q = 'UPDATE amro_trips SET RecievedBy=?,StatusId=? where Id=?;'
    var stmt = dbCursor.prepare(q, [userId, 90, tripId])
    stmt.run();
    stmt.finalize();
    cb(1)
}

module.exports.canceldelivery = (cb, userId, tripId) => {
    var q = 'UPDATE amro_trips SET RecievedBy=?,StatusId=? where Id=?;'
    var stmt = dbCursor.prepare(q, [userId, 120, tripId])
    stmt.run();
    stmt.finalize();
    cb(1)
}

module.exports.update_amro_stock = (cb, addMode, stockId, tripId) => {
    var q = 'select Id,Quantity from amro_stock where StockId=? and TripId=?;'
    var stmt = null
    dbCursor.get(q, [stockId, tripId], (err, stockData) => {
        console.log(stockData)
        if (stockData) {
            var quantity = +stockData.Quantity
            if (addMode) quantity += 1
            else if (quantity > 0) quantity -= 1
            q = 'update amro_stock SET Quantity=? where Id=?'
            stmt = dbCursor.prepare(q, [quantity, stockData.Id])
            console.log("Updated Quantity:" + quantity)
        } else {
            q = "INSERT INTO amro_stock(CreatedOn,StockId,TripId,Quantity) VALUES (?,?,?,?)"
            stmt = dbCursor.prepare(q, [(new Date()).toLocaleTimeString(), stockId, tripId, 1])
        }
        if (stmt) {
            stmt.run();
            stmt.finalize();
            cb(1)
        } else cb(-1)
    })
}

module.exports.setbreaks = (cb, tripId, stop) => {
    var tripStatus = stop ? 130 : 70
    var q = 'UPDATE amro_trips SET StatusId=? where Id=?;'
    var stmt = dbCursor.prepare(q, [tripStatus, tripId])
    stmt.run();
    stmt.finalize();
    cb(1)
}

/*module.exports.get_device_stats = () => {
    self.cursor.execute('SELECT UpdatedOn,Name,Value FROM device_stats')
    return self.cursor.fetchall()
}*/
