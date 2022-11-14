var ipRegex = /([0-9]{1,3}(\.[0-9]{1,3}){3}|[a-f0-9]{1,4}(:[a-f0-9]{1,4}){7})/g;
var webcam_addr = (window.location.href.match(ipRegex) || ["127.0.0.1"])[0];
var webcam_port = "12000";
var webcam_host = $(".feed img");
var socket = io.connect('http://' + webcam_addr + ':' + webcam_port);

socket.on('image', function (data) {
    webcam_host.attr("src", "data:image/jpeg;base64," + data);
});

console.log("touchscreen is", VirtualJoystick.touchScreenAvailable() ? "available" : "not available");

var joystick = new VirtualJoystick({
    container: document.getElementById('container'),
    mouseSupport: true,
});
joystick.addEventListener('touchStart', function () {
    console.log('down')
})
joystick.addEventListener('touchEnd', function () {
    console.log('up')
})

function move(url) {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", url, true);
    xhr.onload = function (e) {
        if (xhr.readyState === 4) {
            if (xhr.status === 200) {
                console.log(xhr.responseText);
            } else {
                console.error(xhr.statusText);
            }
        }
    };
    xhr.onerror = function (e) {
        console.error(xhr.statusText);
    };
    xhr.send(null);
}

var ldx = 0,
    ldy = 0;

setInterval(function () {
    var outputEl = document.getElementById('result');

    var dx = joystick.deltaX(),
        dy = joystick.deltaY();

    outputEl.innerHTML = '<b>Result:</b> ' +
        ' dx:' + dx +
        ' dy:' + dy +
        (joystick.right() ? ' right' : '') +
        (joystick.up() ? ' up' : '') +
        (joystick.left() ? ' left' : '') +
        (joystick.down() ? ' down' : '');


    if (ldx != dx || ldy != dy) {
        move('/move?dx=' + dx + '&dy=' + dy);
        ldx = dx;
        ldy = dy;
    }

}, 1 / 30 * 1000);
