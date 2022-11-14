var express = require('express');
var app = express();
const fs = require('fs');

const {
    exec
} = require('child_process');

var signature_ip_data = ""

app.get('/signature', function (req, res) {
    var k = [];
    var data = signature_ip_data.toString().split("@@@").forEach((a, i) => {
        var b = a.split(",!");
        if (b[0] == "TXTLOCIPNO")
            k.push({});
        else if (k[k.length - 1])
            k[k.length - 1][b[0]] = b[1];
    })
    res.send(JSON.stringify(k))
});

const fetchAllData = () => {
    exec('./scripts/signature/ipdata.sh', (err, stdout, stderr) => {
        if (err) {
            console.error(err)
        } else {
            fs.readFile('./data/signature/ipdata', (err, data) => {
                if (!err) {
                    signature_ip_data = data.toString()
                    console.log("Saved Signature ipdata.")
                }
            });
        }
    });
}

const update = () => {

}

const init = () => {
    app.listen(3000);
    fetchAllData();
    setInterval(update, 1000);
}

init();
