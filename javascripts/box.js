class Box {
	constructor(x, y) {
		this.id = randomString(5)

		this.previousCarAngle = 0;
		this.carAngle = -Math.PI/2; // Car body angle (radians)
		this.pos = new Vector(x || 100, y || canvas.height/2); // Position
		this.previousPosition = this.pos.copy();
		this.velocity = new Vector(0, 0); // Velocity
		this.localVelocity = new Vector();
		this.acceleration = new Vector(); // Acceleration
		this.localAcceleration = new Vector();
		this.absVelocity = 0; // Metres / Second
		this.speed = 0; // KM/hr
		this.yawRate = 0; // Angular velocity
		this.steeringAngle = 0; // Steering angle

		this.setConfig(); // Car configuration
	}

	setConfig(cfg) {
		// Configuration of Car

		this.gravity = 9.81; // Meters / Second
		this.mass = 1200 // Kilograms

		// Body
		this.halfWidth = 0.8;
		this.height = 0.55;
		this.cgToFront = 2.0;
		this.cgToRear = 2.0;   // Centre of gravity to rear of chassis
		this.cgToFrontAxle = 1.25;  // Centre gravity to front axle
		this.cgToRearAxle = 1.25;  // Centre gravity to rear axle
		this.wheelBase = this.cgToFrontAxle + this.cgToRearAxle;
		this.cgHeight = 0.55;  // Centre gravity height
		this.wheelRadius = 0.3;  // Includes tire (also represents height of axle)

		this.w = (this.cgToFront + this.cgToRear)*scale;
		this.h = this.halfWidth*2*scale;
		this.boundingBox = {
			tl: new Vector(-this.w/2, -this.h/2),
			br: new Vector(this.w/2, this.h/2)
		}

		// Wheels (with respect to the car position)
		this.wheels = {};
		this.wheels.size = this.w/4,
		this.wheels.thickness = this.h/4,

		this.wheels.back = new Vector(this.boundingBox.tl.x + this.w/15 + this.wheels.size/2, this.boundingBox.tl.y + this.h/2),
		this.wheels.back.slipAngle = 0;
		this.wheels.front = new Vector(this.boundingBox.tl.x + this.w - this.wheels.size/2 - this.w/15, this.boundingBox.tl.y + this.h/2 + this.wheels.thickness/2 - this.wheels.thickness/2)
		this.wheels.front.slipAngle = 0;

		this.wheels.baseline = dist(this.wheels.back, this.wheels.front);

		this.wheels.particles = []; // Particles for drifting

		this.tireGrip = 2.0;
		this.lockGrip = 0.7;
		this.engineForce = 8000;
		this.brakeForce = 12000;
		this.eBrakeForce = this.brakeForce / 2.5;
		this.weightTransfer = 0.2;
		this.maxSteer = 0.6;
		this.cornerStiffnessFront = 5.0;
		this.cornerStiffnessRear = 5.2;

		this.drag = 2.5; // Air resistance
		this.rrDrag = 12.0; // Rolling resistance
	}

	getBoundingBox(object) {
		if (object) {
			return {
				tl: Vector.rotate(new Vector(-this.w/2, -this.h/2), this.carAngle),
				tr: Vector.rotate(new Vector(this.w/2, -this.h/2), this.carAngle),
				br: Vector.rotate(new Vector(this.w/2, this.h/2), this.carAngle),
				bl: Vector.rotate(new Vector(-this.w/2, this.h/2), this.carAngle)
			};
		} else {
			let result = [
				Vector.rotate(new Vector(-this.w/2, -this.h/2), this.carAngle),
				Vector.rotate(new Vector(this.w/2, -this.h/2), this.carAngle),
				Vector.rotate(new Vector(this.w/2, this.h/2), this.carAngle),
				Vector.rotate(new Vector(-this.w/2, this.h/2), this.carAngle)
			]

			for (let i = 0; i < result.length; i++) {

				result[i].add(this.pos);
			}

			return result;
		}
	}

	applyPhysics() {
		// Set previous car position
		this.previousPosition = this.pos.copy();
		this.previousCarAngle = this.carAngle;

		// Get car local velocity
		this.localVelocity.x = Math.cos(this.carAngle) * this.velocity.x + Math.sin(this.carAngle) * this.velocity.y;
		this.localVelocity.y = Math.cos(this.carAngle) * this.velocity.y - Math.sin(this.carAngle) * this.velocity.x;

		// Weight on axles based on center of gravity and weight shift due to forward/reverse acceleration
		var axleWeightFront = this.mass * (0.5 * this.gravity - this.weightTransfer * this.localAcceleration.x * this.height / this.wheelBase);
		var axleWeightRear = this.mass * (0.5 * this.gravity + this.weightTransfer * this.localAcceleration.x * this.height / this.wheelBase);
		//console.log(axleWeightFront, axleWeightRear);

		var yawSpeedFront = this.cgToFrontAxle * this.yawRate;
		var yawSpeedRear = -this.cgToRearAxle * this.yawRate;

		// Calculate sideslip angle for car
		this.sideslipAngle = (this.carAngle%rad(360) - this.velocity.getDir())%rad(360);

		// Calculate slip angle for front/back wheel
		this.wheels.front.slipAngle = Math.atan2(this.localVelocity.y + yawSpeedFront, Math.abs(this.localVelocity.x)) - Math.sign(this.localVelocity.x) * this.steeringAngle;
		this.wheels.back.slipAngle = Math.atan2(this.localVelocity.y + yawSpeedRear,  Math.abs(this.localVelocity.x));
		
		var tireGripFront = this.tireGrip;
		var tireGripRear = this.tireGrip * (1.0 * (1.0 - this.lockGrip)); // reduce rear grip when ebrake is on

		var frictionForceFront = Math.clamp(-this.cornerStiffnessFront * this.wheels.front.slipAngle, -tireGripFront, tireGripFront) * axleWeightFront;
		var frictionForceRear = Math.clamp(-this.cornerStiffnessRear * this.wheels.back.slipAngle, -tireGripRear, tireGripRear) * axleWeightRear;

		// Frictional forces
		let dragForce = new Vector(this.localVelocity.x * Math.abs(this.localVelocity.x) * -this.drag, this.localVelocity.y * Math.abs(this.localVelocity.y) * -this.drag); // Air resistance force
		let rollingResistanceForce = new Vector(this.localVelocity.x * -this.rrDrag, this.localVelocity.y * -this.rrDrag); // Rolling resistance force (friction with ground)

		 // Total force applied on car
		let netForce = new Vector();
		netForce.add(dragForce);
		netForce.add(rollingResistanceForce);

		// Add cornering force as well
		this.corneringForce = Math.cos(this.steeringAngle) * frictionForceFront + frictionForceRear;
		netForce.y += this.corneringForce;

		// Compute acceleration
		this.localAcceleration.x = netForce.x / this.mass;
		this.localAcceleration.y = netForce.y / this.mass;

		// Convert to global acceleration
		this.acceleration.x = Math.cos(this.carAngle) * this.localAcceleration.x - Math.sin(this.carAngle) * this.localAcceleration.y;
		this.acceleration.y = Math.sin(this.carAngle) * this.localAcceleration.x + Math.cos(this.carAngle) * this.localAcceleration.y;

		// Compute global velocity
		let accelerationDelta = this.acceleration.copy();
		accelerationDelta.mult(dt);
		this.velocity.add(accelerationDelta);

		this.absVelocity = this.velocity.getMag(); // Get speed of velocity in metres per second
		this.speed = this.velocity.getMag() * 3600 / 1000; // Calculate speed in KM/hr

		// Calculate amount of rotational force
		let angularTorque = (frictionForceFront) * this.cgToFrontAxle - frictionForceRear * this.cgToRearAxle; 

		this.absVelocity = this.velocity.getMag(); // Get speed of velocity in metres per second

		// Stop car if speed is negligible
		/*if (this.absVelocity < 0.1) {
			this.velocity = new Vector();
			this.absVelocity = 0;
			angularTorque = 0;
		}*/

		// Calculate car angle from angular torque
		this.inertia = 1/12*this.mass*(Math.pow(this.cgToFront + this.cgToRear, 2) + Math.pow(this.halfWidth*2, 2));

		let angularAcceleration = angularTorque / this.mass;
		this.yawRate += angularAcceleration * dt;
		this.carAngle += this.yawRate * dt;

		// Calculate new car position
		let velocityDelta = this.velocity.copy();
		velocityDelta.mult(dt*scale);
		this.pos.add(velocityDelta);
	}

	collisionDetection() {
		// Out of bounds

		if (this.pos.x > canvas.width + this.w/2) {
			this.pos.x = -this.w/2;
		} else if (this.pos.x < -this.w/2) {
			this.pos.x = canvas.width + this.w/2;
		} else if (this.pos.y > canvas.height + this.h) {
			this.pos.y = -this.h;
		} else if (this.pos.y < -this.h) {
			this.pos.y = canvas.height + this.h;
		}
	}

	move() {
		this.applyPhysics();
		this.collisionDetection();
	}

	addParticles() {
		let particles = this.wheels.particles;

		let ssa = Math.abs(this.sideslipAngle);
		if ((ssa > 0.2 && ssa < Math.PI) || (ssa > Math.PI && 2*Math.PI-ssa > 0.2)) {

			particles.push(
				{
					pos: new Vector(this.pos.x+Math.cos(this.carAngle)*this.wheels.back.x+Math.cos(this.carAngle+rad(90))*this.h/2, this.pos.y+Math.sin(this.carAngle)*this.wheels.back.x+Math.sin(this.carAngle+rad(90))*this.h/2),
					time: Date.now()
				}
			);
			particles.push(
				{
					pos: new Vector(this.pos.x+Math.cos(this.carAngle)*this.wheels.back.x+Math.cos(this.carAngle-rad(90))*this.h/2, this.pos.y+Math.sin(this.carAngle)*this.wheels.back.x+Math.sin(this.carAngle-rad(90))*this.h/2),
					time: Date.now()
				}
			);
			particles.push(
				{
					pos: new Vector(this.pos.x+Math.cos(this.carAngle)*this.wheels.front.x+Math.cos(this.carAngle+rad(90))*this.h/2, this.pos.y+Math.sin(this.carAngle)*this.wheels.front.x+Math.sin(this.carAngle+rad(90))*this.h/2),
					time: Date.now()
				}
			);
			particles.push(
				{
					pos: new Vector(this.pos.x+Math.cos(this.carAngle)*this.wheels.front.x+Math.cos(this.carAngle-rad(90))*this.h/2, this.pos.y+Math.sin(this.carAngle)*this.wheels.front.x-Math.sin(this.carAngle+rad(90))*this.h/2),
					time: Date.now()
				}
			);
		}

		for (var i = particles.length - 1; i >= 0; i--) {
			if (Date.now()-particles[i].time>1000)
				particles.splice(i, 1);
		}
	}

	drawParticles() {
		// DRAW PARTICLES

		let particles = this.wheels.particles;

		for (var j = 0; j < 4; j++) {
			var lastP = new Vector(-1, -1);
			for (var i = j; i < particles.length; i+=4) {
				var p = particles[i].pos;

				var inArena = lastP.x > 0 && lastP.y > 0 && lastP.x < canvas.width && lastP.y < canvas.height && p.x > 0 && p.y > 0 && p.x < canvas.width && p.y < canvas.height;
				var close = dist(lastP, p) < 40;
				if (inArena && close) {
					drawLine(lastP.x, lastP.y, p.x, p.y, this.wheels.thickness*4/5, "black", 1-(Date.now()-particles[i].time)/1000, "round");
				}

				lastP = p;
			}
		}
	}

	drawCar() {
		// DRAW CAR

		ctx.save();
		ctx.translate(this.pos.x, this.pos.y);
		ctx.rotate(this.carAngle);

		// Draw back wheels
		ctx.fillStyle = "black";
		ctx.fillRect(this.boundingBox.tl.x + this.w/15, this.boundingBox.tl.y - this.wheels.thickness/2, this.wheels.size, this.wheels.thickness);
		ctx.fillRect(this.boundingBox.tl.x + this.w/15, this.boundingBox.tl.y + this.h - this.wheels.thickness/2, this.wheels.size, this.wheels.thickness);

		// Draw front wheels (they rotate)
		ctx.fillStyle = "red";
		ctx.save();
			var wheelCenter = new Vector(this.boundingBox.tl.x + this.w - this.wheels.size/2 - this.w/15, this.boundingBox.tl.y + this.wheels.thickness/2 - this.wheels.thickness/2);
			ctx.translate(wheelCenter.x, wheelCenter.y);
			ctx.rotate(this.steeringAngle);

			var wheel = new Vector(-this.wheels.size/2, -this.wheels.thickness/2);
			ctx.fillRect(wheel.x, wheel.y, this.wheels.size, this.wheels.thickness);
		ctx.restore();

		ctx.save();
			var wheelCenter = new Vector(this.boundingBox.tl.x + this.w - this.wheels.size/2 - this.w/15, this.boundingBox.tl.y + this.h + this.wheels.thickness/2 - this.wheels.thickness/2);
			ctx.translate(wheelCenter.x, wheelCenter.y);
			ctx.rotate(this.steeringAngle);

			var wheel = new Vector(-this.wheels.size/2, -this.wheels.thickness/2);
			ctx.fillRect(wheel.x, wheel.y, this.wheels.size, this.wheels.thickness);
		ctx.restore();

		// Draw car body
		ctx.fillStyle = this.color || "green";
		ctx.fillRect(this.boundingBox.tl.x, this.boundingBox.tl.y, this.w, this.h);

		ctx.restore();
	}

	draw() {
		this.drawParticles();

		this.drawCar();
	}
}