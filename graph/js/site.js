var i = 0;
var width, height;
//Just dont look in here
const s = (pi) => {
    p = pi;
    pi.setup = function () {
        pi.createCanvas(p.windowWidth, p.windowHeight);
        width = p.windowWidth;
        height = p.windowHeight;
        $("canvas").contextmenu((e) => {
            e.preventDefault();
        });
    };
    var tMax = data.length;

    var hMax = 0;
    for (var i in data) {
        if (data[i][0] > hMax) {
            hMax = data[i][0];
        }
    }

    pi.draw = function () {
        pi.background(255);
        var lastHeight = 0;
        var lastNoisy = 0;
        var lastKalmanLower = 0;
        var lastKalmanUpper = 0;
        var maxThrust = data[0][2];

        for (var i in data) {
            var d = data[i][0];
            var dn = data[i][1];
            var k = data[i][3];
            // 90% interval, with stddev = data[i][4]
            var kLower = data[i][3] - 1.645 * data[i][4];
            var kUpper = data[i][3] + 1.645 * data[i][4];
            pi.stroke(0);
            pi.line(
                (+i / tMax) * width,
                height - 50 - (d / hMax) * (height - 100),
                ((+i - 1) / tMax) * width,
                height - 50 - (lastHeight / hMax) * (height - 100),
            );
            pi.point(
                (i / tMax) * width,
                height - 50 - (data[i][2] * (height - 100)) / maxThrust,
            );
            pi.stroke(255, 0, 0);
            pi.line(
                (+i / tMax) * width,
                height - 50 - (dn / hMax) * (height - 100),
                ((+i - 1) / tMax) * width,
                height - 50 - (lastNoisy / hMax) * (height - 100),
            );
            pi.stroke(0, 255, 0);
            pi.line(
                (+i / tMax) * width,
                height - 50 - (kLower / hMax) * (height - 100),
                ((+i - 1) / tMax) * width,
                height - 50 - (lastKalmanLower / hMax) * (height - 100),
            );
            pi.stroke(0, 0, 255);
            pi.line(
                (+i / tMax) * width,
                height - 50 - (kUpper / hMax) * (height - 100),
                ((+i - 1) / tMax) * width,
                height - 50 - (lastKalmanUpper / hMax) * (height - 100),
            );

            lastHeight = d;
            lastNoisy = dn;
            lastKalman = k;
            lastKalmanLower = kLower;
            lastKalmanUpper = kUpper;
        }
    };

    pi.mouseDragged = function () {};
    pi.mouseClicked = function () {};
    pi.mouseReleased = function () {};
};
