// Uses the bezier curve library to generate values for the motor controller

var BezierEasing = require('bezier-easing');

var easing = BezierEasing(.35,-0.29,.53,1.54);

function limiter(num) {
    if (num < 0) {
        return 0;
    } else if (num > 100) {
        return 100;
    } else {
        return num;
    }
}

var outputArray = [];
for (var i = 0; i < 1; i = i + 0.01) {
    outputArray.push(limiter(Math.round(easing(i) * 100), 0));
}

var output = "";
output += "{";
outputArray.forEach((elem) => {
    output += String(elem) + ",";
});
output += String(limiter(Math.round(easing(1.0) * 100, 0)));
output += "}";
console.log(output);
console.log(outputArray);
var graph = "";
for (var i = 100/4; i >= 0; i--) {
    for (var j = 0; j < 101; j++) {
        if (outputArray[j] == i*4 || outputArray[j] == (i*4-1) || outputArray[j] == (i*4-2) || outputArray[j] == (i*4-3)) {
            graph += "x";
        } else {
            graph += " ";
        }
    }
    graph += "\n";
}
console.log(graph);
