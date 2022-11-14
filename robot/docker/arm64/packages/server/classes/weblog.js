var fs = require('fs'),
    bite_size = 256,
    readbytes = 0,
    file,
    ws

function readFile(err, fd) {
    if(fd) file = fd
    if(fs.fstatSync(file).size<readbytes+1) setTimeout(readFile, 3000)
    else fs.read(file, new Buffer(bite_size), 0, bite_size, readbytes, readBuff)
}

function readBuff(err, bytecount, buff) {
    //console.log('Read', bytecount, 'and will process it now.')
    var readLine = buff.toString('utf-8', 0, bytecount)
    console.log(readLine)
    readbytes+=bytecount
    process.nextTick(readFile)
}

fs.open('/home/ajay/sar/logs/test', 'r', readFile)
