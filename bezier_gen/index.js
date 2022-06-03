// Uses the bezier curve library to generate values for the motor controller

var BezierEasing = require('bezier-easing');

var easing = BezierEasing(.45,.06,.69,.94);

for (var i = 0; i <= 1.01; i = i + 0.01) {
    console.log(easing(i)*100);
}