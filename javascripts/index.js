var canvas = document.getElementById("canvas");
var ctx = canvas.getContext("2d");
canvas.width = window.innerWidth;
canvas.height = window.innerHeight;

// Settings
var showMenu = true;
var showSettings = false;
var showStats = true;
var showInstructions = true;
var scale = 25; // (scale) px = 1 meter
var useMouse = false;
var smoothSteer = true;

// Create a new car
var car = new Car();
var car2 = new Car(1000, canvas.height/2);

let boxes = [new Box(canvas.width/2, canvas.height/2)/*, new Box(canvas.height/2, canvas.height/4), new Box(canvas.height/2, canvas.height*3/4)*/];
/*for (let i = 0; i < 5; i++){
    boxes.push(new Box(random(canvas.width, 0), random(canvas.height, 0)))
}*/

function getArea(a, b, c) {
    return Math.abs((b.x - a.x)*(c.y-a.y) - (c.x-a.x)*(b.y-a.y))/2;
}

function distPointLine(p, a, b) {

    let A = a.y - b.y;
    let B = b.x - a.x;
    let C = a.x*b.y - b.x*a.y;

    return Math.abs(A*p.x + B*p.y + C)/Math.sqrt(Math.pow(A, 2) + Math.pow(B, 2));
}

// Rectangular collision detection and response
function checkCollision() {
    while (points.length > 1) {
        points.shift();
    }

    // for (let box of boxes) { // Sum the changes in velocity and angular velocity

    //     resolveCollision(car, box, "Car");
    //     resolveCollision(box, car, "Box");
    // }

    let entities = boxes.concat(
        [car, car2]
    )

    for (let u of entities) {
        for (let v of entities) {
            if (u.id != v.id) {
                resolveCollision(u, v);
                resolveCollision(v, u);
            }
        }
    }
}

let points = []; // Points of collision
function resolveCollision(car1, car2, log) {

    let a = car1;
    let b = car2;

    let bba = a.getBoundingBox(); // rotated bounding box 0: tl 1: tr 2: br 3: bl
    let bbb = b.getBoundingBox();
    let area = b.w * b.h; // Get box area

    let p = undefined; // Point of collision
    let n = undefined; // Normal vector of collision

    for (let point of bba) { // Check for collision
        let a1 = getArea(bbb[0], point, bbb[1]);
        let a2 = getArea(bbb[1], point, bbb[2]);
        let a3 = getArea(bbb[2], point, bbb[3]);
        let a4 = getArea(bbb[3], point, bbb[0]);

        if (a1 + a2 + a3 + a4 - area < 5) { // Collision detected
            //stop = true;

            p = point;
            
            let d1 = distPointLine(p, bbb[0], bbb[1])
            let d2 = distPointLine(p, bbb[1], bbb[2])
            let d3 = distPointLine(p, bbb[2], bbb[3])
            let d4 = distPointLine(p, bbb[3], bbb[0])

            // Get collision normal
            let smallest = Math.min(d1, d2, d3, d4);
            if (d1 == smallest) {
                n = Vector.sub(bbb[0], bbb[1]);
            } else if (d2 == smallest) {
                n = Vector.sub(bbb[1], bbb[2]);
            } else if (d3 == smallest) {
                n = Vector.sub(bbb[2], bbb[3]);
            } else if (d4 == smallest) {
                n = Vector.sub(bbb[3], bbb[0]);
            }

            n.normalize();
            n = Vector.rotate(n, Math.PI/2);
            n = [n.x, n.y]

            // Mass
            let ma = a.mass;
            let mb = b.mass;

            // Relative vector from center of body to point
            let rap_ = Vector.sub(p, a.pos);
            let rbp_ = Vector.sub(p, b.pos);
            rap_ = [rap_.x, rap_.y];
            rbp_ = [rbp_.x, rbp_.y];
            
            points.push([p, n, a.pos.copy(), rap_, b.pos.copy(), rbp_]);

            let rap = Vector.sub(p, a.pos);
            let rbp = Vector.sub(p, b.pos);
            rap = [rap.x/scale, rap.y/scale];
            rbp = [rbp.x/scale, rbp.y/scale];

            // Angular velocity
            let wa1 = a.yawRate;
            let wb1 = b.yawRate;

            // Linear velocity
            let va1 = a.velocity.copy();
            let vb1 = b.velocity.copy();

            // Restitution coefficient
            let e = 1;

            // Velocities with respect to the point of contact
            let vap1 = [va1.x + wa1 * rap[0], va1.y + wa1 * rap[1]];
            let vbp1 = [vb1.x + wb1 * rbp[0], vb1.y + wb1 * rbp[1]];

            // Relative velocity of both bodies
            let vab1 = math.subtract(vap1, vbp1);

            // Impulse
            let jNum = -(1+e)*math.dot(vab1, n);

            // Moment of inertia
            let ia = ma/12*( Math.pow(a.cgToFront + a.cgToRear, 2) + Math.pow(a.halfWidth*2, 2) );
            let ib = mb/12*( Math.pow(b.cgToFront + b.cgToRear, 2) + Math.pow(b.halfWidth*2, 2) );

            rap_perp = Vector.rotate(new Vector(rap[0], rap[1]), Math.PI/2);
            rbp_perp = Vector.rotate(new Vector(rbp[0], rbp[1]), Math.PI/2);
            rap_perp = [rap_perp.x, rap_perp.y];
            rbp_perp = [rbp_perp.x, rbp_perp.y];
            rap_perp = [-rap[1], rap[0]];
            rbp_perp = [rbp[1], -rbp[0]];

            let jDiv1 = (1/ma + 1/mb);
            let jDiv2 = math.dot(rap, n)*math.dot(rap, n)/ia;
            let jDiv3 = math.dot(rbp, n)*math.dot(rbp, n)/ib;

            let jDiv = jDiv1 + jDiv2 + jDiv3;

            let j = jNum/jDiv;

            let j_a = math.multiply(j/ma, n);
            let j_b = math.multiply(j/mb, n);

            let yawRateDeltaA = math.dot(rap_perp, math.multiply(j, n)) / ia;
            let yawRateDeltaB = math.dot(rbp_perp, math.multiply(j, n)) / ib;

            a.color = "red";
            b.color = "red";

            // let dp = 4;
            // console.log(
            //     "Type: " + log + 
            //     "\nJ: " + round(j, dp) +
            //     " \nV_a: " + round(a.velocity.x, dp) + " " + round(a.velocity.y, dp) + " V_b: " + round(b.velocity.x, dp) + " " + round(b.velocity.y, dp)
            //     + " \nJ_a: " + round(j_a[0], dp) + " " + round(j_a[1], dp) + " J_b: " + round(j_b[0], dp) + " " + round(j_b[1], dp) + " \nJ_div: " + round(jDiv, dp) + " J_num: " + round(jNum, dp) + " J_div1: " + round(jDiv1, dp) + " J_div2: " + round(jDiv2, dp) + " J_div3: " + round(jDiv3, dp) + " \nrap: " + round(rap[0], dp) + " " + round(rap[1], dp) + " rbp: " + round(rbp[0], dp) + " " + round(rbp[1], dp) + " \nn: " + round(n[0], dp) + " " + round(n[1], dp) + " \nwa: " + round(wa1, dp) + " wb1: " + round(wb1, dp) + " \nwa_: " + round(yawRateDeltaA, dp) + " wb_: " + round(yawRateDeltaB, dp)
            // );

            a.velocity.x += j_a[0];
            a.velocity.y += j_a[1];

            b.velocity.x -= j_b[0];
            b.velocity.y -= j_b[1];

            if (Math.abs(a.yawRate+yawRateDeltaA) < 8) a.yawRate += yawRateDeltaA;
            if (Math.abs(b.yawRate+yawRateDeltaB) < 8) b.yawRate += yawRateDeltaB;

            // a.yawRate += yawRateDeltaA;    
            // b.yawRate += yawRateDeltaB;

        }
    }

}

function drawStats() {
    // Add statistics
    statsCounter = 0;
    addStat("FPS", 1000/delta, 1, fpsArray);
    addStat("Throttle", car.throttle, 2);
    addStat("Brake", car.brake, 2);
    addStat("Speed", car.speed, 1, speedArray);
    addStat("X", car.pos.x);
    addStat("Y", car.pos.y);
    addStat("A_X", car.acceleration.x);
    addStat("A_Y", car.acceleration.y);
    addStat("Steer Angle", deg(car.steeringAngle));
    addStat("Car Angle", deg(car.carAngle)%360);
    addStat("Yaw rate", car.yawRate, 3);
    addStat("Sideslip", car.sideslipAngle, 2);
    addStat("Cornering Force", car.corneringForce);
}

function drawControls() {
    var ctrl = car.controls;
    if (useMouse) {

        drawText("Full Throttle", ctrl.padding, ctrl.fullPosition-ctrl.padding, "15px Arial", "black", "left", "bottom");
        drawLine(0, ctrl.fullPosition, 100, ctrl.fullPosition, 3, "black", 0.2);

        drawText("Throttle", ctrl.padding, ctrl.idlePosition-ctrl.padding, "15px Arial", "black", "left", "bottom");
        drawLine(0, ctrl.idlePosition, 100, ctrl.idlePosition, 3, "black", 0.2);

        drawText("Idle", ctrl.padding, ctrl.idlePosition+(ctrl.brakePosition-ctrl.idlePosition)/2, "15px Arial", "black", "left", "middle");

        drawLine(0, ctrl.brakePosition, 100, ctrl.brakePosition, 3, "black", 0.2);
        drawText("Brake", ctrl.padding, ctrl.brakePosition+ctrl.padding, "15px Arial", "black", "left", "top");

        drawLine(0, canvas.height-ctrl.fullPosition, 100, canvas.height-ctrl.fullPosition, 3, "black", 0.2);
        drawText("Full Brake", ctrl.padding, canvas.height-ctrl.fullPosition+ctrl.padding, "15px Arial", "black", "left", "top");

        drawLine(ctrl.steerLeftPosition, 0, ctrl.steerLeftPosition, 50, 3, "black", 0.2);
        drawText("Steer Left", ctrl.steerLeftPosition-ctrl.padding, ctrl.padding, "15px Arial", "black", "right", "top");
        drawLine(ctrl.steerRightPosition, 0, ctrl.steerRightPosition, 50, 3, "black", 0.2);
        drawText("Steer Right", ctrl.steerRightPosition+ctrl.padding, ctrl.padding, "15px Arial", "black", "left", "top");

        drawText("Right Mouse Button - emergency brake", canvas.width/2, canvas.height-25, "", "black", "center", "top");
    } else {
        drawText("W / Up - accelerate, S / Down - brake, A / D / Left / Right - steer, Space - emergency brake", canvas.width/2, canvas.height-25, "", "black", "center", "top");
    }
}

function drawMenu() {
    drawText("Car Simulator", canvas.width/2, canvas.height/2, canvas.height/8 + "px Arial", "black", "center", "bottom");
    drawText("Developed by Victor Wei", canvas.width/2, canvas.height/2, canvas.height/32+"px Arial", "black", "center", "top");

    if (car.speed > 20 || car2.speed > 20)
        showMenu = false;
}

var stop = false;
var unstopToggle = false;
var unstopDelay = Date.now();

function update() {

    if (map[84] && !unstopToggle) {
        console.log("Unstopped");
        stop = false;
        unstopToggle = true;
        unstopDelay = Date.now();

    } else if ((!map[84] && unstopToggle)) {
        unstopToggle = false;
    } else if (map[84] && Date.now() - unstopDelay > 500) {
        stop = false;
    }

    if (stop) {
        return;
    }
    car.move(0);
    car.addParticles();
    car.color = "blue";

    car2.move(1);
    car2.addParticles();
    car2.color = "orange";

    for (let box of boxes) {
        box.move();
        box.addParticles();
        box.color = "green";
    }

    checkCollision();

    if (map[82]) {
        car = new Car();
        car2 = new Car(1000, canvas.height/2);
        boxes = [new Box(canvas.width/2, canvas.height/2)/*, new Box(canvas.height/2, canvas.height/4), new Box(canvas.height/2, canvas.height*3/4)*/];
        points = [];
    }
}

function draw() {
    ctx.fillStyle = "#EAEDED";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    car.draw();
    car2.draw();

    for (let box of boxes) {
        box.draw();
    }

    if (showStats) {
        let index = 0
        for (let [p, n, aPos, rap, bPos, rbp] of points) {
            let opacity = 1//index/points.length;
            drawCircle(p.x, p.y, 5, opacity, "lime") // Draw point of collision
            drawArrow(p.x, p.y, p.x+n[0]*100, p.y+n[1]*100, 1, "black", opacity) // Draw normal vector

            drawArrow(aPos.x, aPos.y, aPos.x + rap[0], aPos.y + rap[1], 2, "yellow", opacity) // Draw vector to point of collision
            drawArrow(bPos.x, bPos.y, bPos.x + rbp[0], bPos.y + rbp[1], 2, "blue", opacity) // Draw vector to point of collision
            index += 1;
        }
    }

    if (showMenu)
        drawMenu();

    if (showStats)
        drawStats();

    if (showInstructions)
        drawControls();
}

// Stats to track
var statsCounter = 0;
function addStat(name, value, decimalPlace, averageArray, maxCount) {
    let displayValue = value;
    if (averageArray) {
        averageArray.push(value);
        if (averageArray.length > (maxCount || 30))
            averageArray.shift();
        displayValue = averageArray.reduce((a, b) => a + b, 0)/averageArray.length;
    }

    drawText(name + ": " + round(displayValue, decimalPlace), canvas.width-10, 10 + 25*statsCounter, "", "", "right", "top")

    statsCounter += 1;
}

// Stats

var fps = 1000;
var now;
var then = Date.now();
var interval = 1000/fps;
var delta, dt;

var fpsArray, speedArray;
fpsArray = [];
speedArray = [];

// Game loop

function loop() {
    //requestAnimationFrame(loop);

    now = Date.now();
    delta = now - then;
    dt = delta/1000;

    if (delta > interval) {
        then = now - (delta % interval);

        ctx.clearRect(0, 0, canvas.width, canvas.height);
        update();
        draw();
    }
}
//loop();

setInterval(loop, interval);

var map = {};
onkeydown = onkeyup = function(e){
    e = e || event; 
    map[e.keyCode] = e.type == 'keydown';
}

var mouse = new Vector();
$("#canvas").on("mousemove", function (e) {
    mouse.x = e.offsetX;
    mouse.y = e.offsetY;
})

$("#canvas").on('mousedown', function (e) {
    if (e.which == 1) {
        mouse.left = true;
    } else if (e.which == 2) {
        mouse.middle = true;
    } else if (e.which == 3) {
        mouse.right = true;
    }
    e.preventDefault();
}).on('mouseup', function (e) {
    if (e.which == 1) {
        mouse.left = false;
    } else if (e.which == 2) {
        mouse.middle = false;
    } else if (e.which == 3) {
        mouse.right = false;
    }
    e.preventDefault();
})

$("#settings-image").click(function () {
    showSettings = !showSettings;

    if (showSettings) {
        $("#settings").show();
    } else {
        $("#settings").hide();
    }
})

$("#statsSlider").change(function () {
    showStats = !showStats;
})

$("#instructionsSlider").change(function () {
    showInstructions = !showInstructions;
})

$("#controlsSlider").change(function () {
    useMouse = !useMouse;

    if (useMouse) {
        $("#controlsText").text("Controls: Mouse");
    } else {
        $("#controlsText").text("Controls: Keyboard");
    }
})

window.oncontextmenu = function () {
    return false;
}

$(window).resize(function () {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
})