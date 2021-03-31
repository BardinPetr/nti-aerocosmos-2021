var child_process = require('child_process');
var Avrgirl = require('avrgirl-arduino');
var bonjour = require('bonjour')();
var express = require('express');
var fs = require('fs');


bonjour.publish({
    name: 'OTAUpdater',
    type: 'http',
    txt: {
        sys: "H2Pcs::OTA"
    },
    port: 4132
})

var app = express();

app.get('/flash/:hex', function (req, res) {
    child_process.spawn('pm2 stop all', []);
    var code = Buffer.from(req.params.hex, 'base64').toString('utf-8');
    var path = 'tempOTA.hex';
    fs.writeFile(path, code, function (err) {
        if (err) {
            res.send(err.toString());
            child_process.spawn('pm2 start all', []);
            return console.log(err);
        }
        var avrgirl = new Avrgirl({
            board: 'uno',
            port: process.argv[2] || process.argv[3]
        });
        avrgirl.flash(path, function (error) {
            if (error) {
                res.send(error.toString());
                console.error(error);
            } else {
                res.send("OK");
                console.info('done.');
            }
            fs.unlink(path);
            child_process.spawn('pm2 start all', []);
        });
    });
});

app.listen(4132, function () {
    console.log('OTAUpdater is listening on port 4132!');
});